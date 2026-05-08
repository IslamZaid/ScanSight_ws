#!/usr/bin/python3
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

EVENT_TOPIC = "/capture_node/events"
BAG_NAME_PREFIX = "events_recording"
EVENT_TOPIC_WAIT_SECONDS = 10.0
RECORD_CONFIRM_SECONDS = 2.0
ROSBAG_BUFFER_MB = 1024


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
            topic,
            rospy.AnyMsg,
            self._callback,
            queue_size=100,
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
            message_count, _, _ = self.snapshot()
            if message_count >= minimum_count:
                return True
            time.sleep(0.05)

        return False

    def stop(self):
        self._subscriber.unregister()


def get_published_topic_type(topic):
    for topic_name, topic_type in rospy.get_published_topics():
        if topic_name == topic:
            return topic_type

    return None


def print_event_topic_suggestions():
    candidates = []
    for topic_name, topic_type in rospy.get_published_topics():
        topic_name_lower = topic_name.lower()
        if "event" in topic_name_lower or "capture_node" in topic_name_lower:
            candidates.append((topic_name, topic_type))

    if not candidates:
        print("No event/capture topics are currently published.")
        return

    print("Available event/capture topics:")
    for topic_name, topic_type in candidates:
        print(f"  {topic_name} ({topic_type})")


def wait_for_event_stream(topic, timeout):
    topic_type = get_published_topic_type(topic)
    if topic_type is None:
        print(f"Waiting for event camera topic: {topic}")
    else:
        print(f"Waiting for event camera messages on {topic} ({topic_type})")

    monitor = EventStreamMonitor(topic)
    try:
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if monitor.wait_for_messages(1, 0.5):
                topic_type = get_published_topic_type(topic) or "unknown type"
                print(f"Event stream is live: {topic} ({topic_type})")
                return True

            topic_type = get_published_topic_type(topic)
            if topic_type is not None:
                print(f"Topic exists, waiting for event messages: {topic} ({topic_type})")
    finally:
        monitor.stop()

    print(f"Error: no event messages received on {topic} within {timeout:.1f} seconds.")
    print_event_topic_suggestions()
    return False


def get_bag_topic_message_count(bag_path, topic):
    try:
        with rosbag.Bag(bag_path, "r") as bag:
            return bag.get_message_count(topic)
    except Exception as exc:
        print(f"Warning: could not inspect saved bag: {exc}")
        return None


def start_event_recording(output_dir):
    """
    Start rosbag recording for the event topic.
    Returns the recording session info.
    """
    session = build_recording_session(output_dir)
    cmd = [
        "rosbag", "record",
        "-q",
        "-O", session["temp_bag"],
        "-b", str(ROSBAG_BUFFER_MB),
        "--tcpnodelay",
        EVENT_TOPIC,
    ]

    print(f"Starting event recording: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        preexec_fn=os.setsid  # lets us stop the whole process group cleanly
    )
    session["proc"] = proc

    time.sleep(0.5)
    if proc.poll() is not None:
        raise RuntimeError("rosbag record exited immediately. Check ROS master and topic name.")

    monitor = EventStreamMonitor(EVENT_TOPIC)
    session["monitor"] = monitor
    if monitor.wait_for_messages(1, RECORD_CONFIRM_SECONDS):
        message_count, byte_count, _ = monitor.snapshot()
        print(
            "Event stream is active during recording: "
            f"{message_count} message batch(es), {byte_count} serialized bytes observed."
        )
    else:
        print(f"Warning: no event messages observed during the first {RECORD_CONFIRM_SECONDS:.1f} seconds.")

    return session


def stop_event_recording(session):
    """
    Stop rosbag recording cleanly.
    """
    if session is None:
        return

    proc = session["proc"]
    monitor = session.get("monitor")

    print("Stopping event recording...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
        print("Recording stopped cleanly.")
    except Exception as e:
        print(f"Warning: clean stop failed: {e}")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass

    if monitor is not None:
        message_count, byte_count, _ = monitor.snapshot()
        monitor.stop()
        print(
            "Observed event stream while recording: "
            f"{message_count} message batch(es), {byte_count} serialized bytes."
        )

    end_time = bag_timestamp()
    final_bag = os.path.join(
        os.path.dirname(session["temp_bag"]),
        f"{BAG_NAME_PREFIX}_{session['number']:03d}_"
        f"start-{session['start_time']}_end-{end_time}.bag"
    )

    if os.path.exists(session["temp_bag"]):
        os.rename(session["temp_bag"], final_bag)
        print(f"Saved event recording to: {final_bag}")

        saved_message_count = get_bag_topic_message_count(final_bag, EVENT_TOPIC)
        if saved_message_count is None:
            return
        if saved_message_count == 0:
            print(f"Warning: saved bag has 0 messages on {EVENT_TOPIC}.")
        else:
            print(f"Saved bag contains {saved_message_count} message(s) on {EVENT_TOPIC}.")
    elif os.path.exists(session["temp_bag"] + ".active"):
        print(f"Warning: rosbag is still active at: {session['temp_bag']}.active")
    else:
        print(f"Warning: expected rosbag was not found: {session['temp_bag']}")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    save_file = os.path.join(script_dir, "saved_poses.json")
    # save_file = os.path.join(script_dir, "office_pose.json")
    bag_dir = os.path.join(script_dir, "recordings")
    os.makedirs(bag_dir, exist_ok=True)

    if not os.path.exists(save_file):
        print(f"Error: Could not find {save_file}")
        return

    print(f"Loading poses from {save_file}...")
    with open(save_file, "r") as f:
        recorded_poses = json.load(f)

    print(f"Loaded {len(recorded_poses)} poses successfully.")

    robot_ip = "192.168.50.110"
    print(f"\nConnecting to UR10 at {robot_ip}...")
    try:
        robot = UrRtde(robot_ip)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("Successfully connected to the robot!")

    rospy.init_node('play_saved_poses', anonymous=True)
    if not wait_for_event_stream(EVENT_TOPIC, EVENT_TOPIC_WAIT_SECONDS):
        print("Cannot start recording because the event camera stream is not publishing.")
        return

    vel = 0.05
    acc = 0.1

    recording_session = None

    print("\nWARNING: The robot will now move through the loaded points.")
    input("Stand clear and press Enter to execute (or Ctrl+C to cancel)...")

    print("\nPlaying back trajectory...")
    try:
        for i, pose in enumerate(recorded_poses):
            print(f"Moving to Pose {i+1}...")

            # Start recording before pose 2
            if i == 2 and recording_session is None:
                recording_session = start_event_recording(bag_dir)
                time.sleep(5.0)  # give rosbag a moment to start

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

if __name__ == "__main__":
    main()
