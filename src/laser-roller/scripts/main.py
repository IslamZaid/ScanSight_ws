#!/usr/bin/python3
"""
Combined UR10 + Event Camera control script.
  1) Record new poses (recording mode)
  2) Play saved poses (robot + camera pipeline)
  3) Camera-only recording (no robot)
"""
import time
import json
import os
import signal
import subprocess
import glob
import re
import threading
from datetime import datetime
import rospy
import rosbag
from ur_rtde import UrRtde

# ---------------------------------------------------------------------------
#  Configuration
# ---------------------------------------------------------------------------
EVENT_TOPIC = "/event_crop/events"
RAW_EVENT_TOPIC = "/capture_node/events"
BAG_NAME_PREFIX = "events_recording"
EVENT_TOPIC_WAIT_SECONDS = 10.0
RECORD_CONFIRM_SECONDS = 2.0
ROSBAG_BUFFER_MB = 1024

CROP_X_MIN = 0
CROP_X_MAX = 100
CROP_Y_MIN = 0
CROP_Y_MAX = 480

CATKIN_SETUP = "/home/iz/ur10_ws/devel/setup.bash"
ROBOT_IP = "192.168.50.110"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POSES_FILE = os.path.join(SCRIPT_DIR, "saved_poses.json")
BAG_DIR = os.path.join(SCRIPT_DIR, "recordings")


# ---------------------------------------------------------------------------
#  Helpers — recording infrastructure
# ---------------------------------------------------------------------------
def bag_timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def next_recording_number(output_dir):
    pattern = os.path.join(output_dir, f"{BAG_NAME_PREFIX}_*.bag")
    highest_number = 0
    for bag_path in glob.glob(pattern):
        match = re.match(rf"{BAG_NAME_PREFIX}_(\d+)_", os.path.basename(bag_path))
        if match:
            highest_number = max(highest_number, int(match.group(1)))
    return highest_number + 1


def build_recording_session(output_dir):
    recording_number = next_recording_number(output_dir)
    start_time = bag_timestamp()
    temp_name = (
        f"{BAG_NAME_PREFIX}_{recording_number:03d}_"
        f"start-{start_time}_end-pending.bag"
    )
    return {
        "number": recording_number,
        "start_time": start_time,
        "temp_bag": os.path.join(output_dir, temp_name),
    }


class EventStreamMonitor:
    def __init__(self, topic):
        self.topic = topic
        self.message_count = 0
        self.byte_count = 0
        self.last_message_time = None
        self._lock = threading.Lock()
        self._subscriber = rospy.Subscriber(
            topic, rospy.AnyMsg, self._callback, queue_size=100,
        )

    def _callback(self, msg):
        with self._lock:
            self.message_count += 1
            self.byte_count += len(getattr(msg, "_buff", b""))
            self.last_message_time = time.time()

    def snapshot(self):
        with self._lock:
            return self.message_count, self.byte_count, self.last_message_time

    def wait_for_messages(self, minimum_count, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            count, _, _ = self.snapshot()
            if count >= minimum_count:
                return True
            time.sleep(0.05)
        return False

    def stop(self):
        self._subscriber.unregister()


def wait_for_event_stream(topic, timeout):
    monitor = EventStreamMonitor(topic)
    try:
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if monitor.wait_for_messages(1, 0.5):
                return True
            time.sleep(0.05)
    finally:
        monitor.stop()
    return False


def get_bag_topic_message_count(bag_path, topic):
    try:
        with rosbag.Bag(bag_path, "r") as bag:
            return bag.get_message_count(topic)
    except Exception as exc:
        print(f"Warning: could not inspect saved bag: {exc}")
        return None


def start_event_recording(output_dir):
    session = build_recording_session(output_dir)
    cmd = [
        "rosbag", "record", "-q",
        "-O", session["temp_bag"],
        "-b", str(ROSBAG_BUFFER_MB),
        "--tcpnodelay", EVENT_TOPIC,
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, preexec_fn=os.setsid)
    session["proc"] = proc
    time.sleep(0.5)
    if proc.poll() is not None:
        raise RuntimeError("rosbag record exited immediately.")
    monitor = EventStreamMonitor(EVENT_TOPIC)
    session["monitor"] = monitor
    monitor.wait_for_messages(1, RECORD_CONFIRM_SECONDS)
    return session


def stop_event_recording(session):
    if session is None:
        return
    proc = session["proc"]
    monitor = session.get("monitor")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    if monitor is not None:
        monitor.stop()
    end_time = bag_timestamp()
    final_bag = os.path.join(
        os.path.dirname(session["temp_bag"]),
        f"{BAG_NAME_PREFIX}_{session['number']:03d}_"
        f"start-{session['start_time']}_end-{end_time}.bag"
    )
    if os.path.exists(session["temp_bag"]):
        os.rename(session["temp_bag"], final_bag)
        saved_count = get_bag_topic_message_count(final_bag, EVENT_TOPIC)
        count_str = f" ({saved_count} msgs)" if saved_count else ""
        print(f"Saved: {os.path.basename(final_bag)}{count_str}")
    elif os.path.exists(session["temp_bag"] + ".active"):
        print(f"Warning: rosbag is still active at: {session['temp_bag']}.active")
    else:
        print(f"Warning: expected rosbag was not found: {session['temp_bag']}")


# ---------------------------------------------------------------------------
#  Helpers — subprocess management
# ---------------------------------------------------------------------------
def launch_subprocess(cmd, label):
    bash_cmd = f"source {CATKIN_SETUP} && {' '.join(cmd)}"
    proc = subprocess.Popen(
        ["bash", "-c", bash_cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    time.sleep(1.0)
    if proc.poll() is not None:
        print(f"  ✗ {label} failed")
        return None
    print(f"  ✓ {label}")
    return proc


def stop_subprocess(proc, label):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
    print(f"  {label} stopped.")


# ---------------------------------------------------------------------------
#  Camera pipeline (shared by modes 2 & 3)
# ---------------------------------------------------------------------------
def start_camera_pipeline():
    """Launch camera driver, crop node, visualization, and rqt.
    Returns list of (label, proc) tuples."""
    print(f"\nStarting camera pipeline...")
    print(f"  Crop: x=[{CROP_X_MIN},{CROP_X_MAX}) y=[{CROP_Y_MIN},{CROP_Y_MAX})")
    procs = []

    # Camera driver
    p = launch_subprocess([
        "rosrun", "dv_ros_capture", "capture_node",
        "__name:=capture_node",
    ], "capture_node")
    procs.append(("capture_node", p))

    # Wait for raw stream
    if not wait_for_event_stream(RAW_EVENT_TOPIC, EVENT_TOPIC_WAIT_SECONDS):
        print("  ✗ Camera not detected. Is it connected?")
        stop_camera_pipeline(procs)
        return None
    print("  ✓ Camera stream live")

    # Event crop
    p = launch_subprocess([
        "rosrun", "event_crop", "event_crop_node.py",
        f"_input_topic:={RAW_EVENT_TOPIC}",
        f"_output_topic:={EVENT_TOPIC}",
        f"_x_min:={CROP_X_MIN}", f"_x_max:={CROP_X_MAX}",
        f"_y_min:={CROP_Y_MIN}", f"_y_max:={CROP_Y_MAX}",
        "_shift_coordinates:=true",
        "__name:=event_crop_node",
    ], "event_crop_node")
    procs.append(("event_crop_node", p))

    # Visualization — full frame
    p = launch_subprocess([
        "rosrun", "dv_ros_visualization", "visualization_node",
        "/raw_viz_node/events:=" + RAW_EVENT_TOPIC,
        "__name:=raw_viz_node",
    ], "raw_viz_node (full frame)")
    procs.append(("raw_viz_node", p))

    # Visualization — cropped
    p = launch_subprocess([
        "rosrun", "dv_ros_visualization", "visualization_node",
        "/crop_viz_node/events:=" + EVENT_TOPIC,
        "__name:=crop_viz_node",
    ], "crop_viz_node (cropped)")
    procs.append(("crop_viz_node", p))

    # rqt — full
    p = launch_subprocess([
        "rosrun", "rqt_image_view", "rqt_image_view",
        "/raw_viz_node/image",
    ], "rqt_image_view (full)")
    procs.append(("rqt_image_view (full)", p))

    # rqt — cropped
    p = launch_subprocess([
        "rosrun", "rqt_image_view", "rqt_image_view",
        "/crop_viz_node/image",
    ], "rqt_image_view (cropped)")
    procs.append(("rqt_image_view (cropped)", p))

    # Confirm cropped stream
    if not wait_for_event_stream(EVENT_TOPIC, EVENT_TOPIC_WAIT_SECONDS):
        print("  ✗ Cropped stream not publishing.")
        stop_camera_pipeline(procs)
        return None
    print("  ✓ Cropped stream live")

    print("\nPipeline ready! Waiting for rqt to load...")
    time.sleep(3)
    return procs


def stop_camera_pipeline(procs):
    if not procs:
        return
    print("\nShutting down pipeline nodes...")
    for label, proc in procs:
        stop_subprocess(proc, label)


# ---------------------------------------------------------------------------
#  Mode 1 — Record new poses (teach mode)
# ---------------------------------------------------------------------------
def mode_record_poses():
    print(f"\nConnecting to UR10 at {ROBOT_IP}...")
    try:
        robot = UrRtde(ROBOT_IP)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return
    print("  ✓ Robot connected\n")

    print("TEACH MODE")
    print("  1. Move the robot with the teach pendant (Free-Drive).")
    print("  2. Press ENTER to record its current pose.")
    print("  3. Type 'q' when finished.\n")

    recorded_poses = []
    try:
        while True:
            user_input = input(
                f"Move to Pose {len(recorded_poses)+1} and press ENTER (or 'q' to finish): "
            )
            if user_input.strip().lower() in ['q', 'quit', 'done', 'exit']:
                break
            current_pose = list(robot.get_pose())
            recorded_poses.append(current_pose)
            print(f"  --> Saved Pose {len(recorded_poses)}: {current_pose}\n")

        if not recorded_poses:
            print("No poses recorded.")
            return

        print(f"\nRecorded {len(recorded_poses)} poses.")
        with open(POSES_FILE, "w") as f:
            json.dump(recorded_poses, f, indent=4)
        print(f"Saved to: {POSES_FILE}")

        playback = input("\nPlay back these poses now? (y/N): ").strip().lower()
        if playback in ['y', 'yes']:
            vel, acc = 0.2, 0.2
            input("\nStand clear and press Enter to execute...")
            print("\nPlaying back trajectory...")
            for i, pose in enumerate(recorded_poses):
                print(f"  Moving to Pose {i+1}/{len(recorded_poses)}...")
                robot.move_TCP(pose, vel, acc)
                time.sleep(0.5)
            print("\nPlayback complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Disconnecting from robot...")
        robot.disconnect()
        print("Disconnected.")


# ---------------------------------------------------------------------------
#  Mode 2 — Play saved poses (robot + camera)
# ---------------------------------------------------------------------------
def mode_play_saved_poses():
    os.makedirs(BAG_DIR, exist_ok=True)

    if not os.path.exists(POSES_FILE):
        print(f"Error: Could not find {POSES_FILE}")
        return

    with open(POSES_FILE, "r") as f:
        recorded_poses = json.load(f)
    print(f"Loaded {len(recorded_poses)} poses from {os.path.basename(POSES_FILE)}")

    print(f"Connecting to UR10 at {ROBOT_IP}...")
    try:
        robot = UrRtde(ROBOT_IP)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return
    print("  ✓ Robot connected")

    rospy.init_node('laser_roller_main', anonymous=True)

    pipeline_procs = start_camera_pipeline()
    if pipeline_procs is None:
        return

    vel, acc = 0.05, 0.1
    recording_session = None

    print("\n--------------------------------------------------")
    print(f"  {len(recorded_poses)} poses loaded. Robot will move through them.")
    input("  Stand clear and press Enter to execute (Ctrl+C to cancel) ")
    print("--------------------------------------------------")

    print("\nPlaying back trajectory...")
    try:
        for i, pose in enumerate(recorded_poses):
            print(f"  Moving to Pose {i+1}/{len(recorded_poses)}...")

            # Start recording before pose 2
            if i == 2 and recording_session is None:
                recording_session = start_event_recording(BAG_DIR)
                time.sleep(5.0)

            robot.move_TCP(pose, vel, acc)

            # Stop recording after pose 3
            if i == 3 and recording_session is not None:
                stop_event_recording(recording_session)
                recording_session = None

            time.sleep(0.5)

        print("\nPlayback complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        if recording_session is not None:
            stop_event_recording(recording_session)
        stop_camera_pipeline(pipeline_procs)


# ---------------------------------------------------------------------------
#  Mode 3 — Camera-only recording (no robot)
# ---------------------------------------------------------------------------
def mode_camera_only():
    os.makedirs(BAG_DIR, exist_ok=True)

    rospy.init_node('laser_roller_main', anonymous=True)

    print("CAMERA-ONLY RECORDING MODE")
    print(f"  Output: {BAG_DIR}")

    pipeline_procs = start_camera_pipeline()
    if pipeline_procs is None:
        return

    recording_session = None

    try:
        while not rospy.is_shutdown():
            print("\n--------------------------------------------------")
            input("  Press Enter to START recording (Ctrl+C to quit) ")
            print("--------------------------------------------------")

            recording_session = start_event_recording(BAG_DIR)
            print("\n>>> RECORDING IN PROGRESS <<<")

            input("Press Enter to STOP recording (or Ctrl+C to stop and quit)...")

            stop_event_recording(recording_session)
            recording_session = None

            again = input("\nRecord another? (y/N): ").strip().lower()
            if again not in ['y', 'yes']:
                break

        print("\nDone. Goodbye!")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    finally:
        if recording_session is not None:
            stop_event_recording(recording_session)
        stop_camera_pipeline(pipeline_procs)


# ---------------------------------------------------------------------------
#  Main — mode selection
# ---------------------------------------------------------------------------
def main():
    print("==================================================")
    print("   UR10 + Event Camera Control")
    print("==================================================")
    print("  1) Record new poses (recording mode)")
    print("  2) Play saved poses (robot + camera)")
    print("  3) Camera-only recording (no robot)")
    print("==================================================")

    # choice = input("\nSelect mode [1/2/3]: ").strip()
    choice = '2'

    if choice == '1':
        mode_record_poses()
    elif choice == '2':
        mode_play_saved_poses()
    elif choice == '3':
        mode_camera_only()
    else:
        print(f"Invalid choice: '{choice}'")


if __name__ == "__main__":
    main()
