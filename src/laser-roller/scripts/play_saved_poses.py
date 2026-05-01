#!/usr/bin/python3
import time
import json
import os
import signal
import subprocess
import rospy
from ur_rtde import UrRtde

def start_event_recording(output_bag):
    """
    Start rosbag recording for the event topic.
    Returns the subprocess handle.
    """
    cmd = [
        "rosbag", "record",
        "/capture_node/events",
        "-O", output_bag
    ]

    print(f"Starting event recording: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid  # lets us stop the whole process group cleanly
    )
    return proc

def stop_event_recording(proc):
    """
    Stop rosbag recording cleanly.
    """
    if proc is None:
        return

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

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    save_file = os.path.join(script_dir, "saved_poses.json")
    bag_file = os.path.join(script_dir, "events_recording.bag")

    if not os.path.exists(save_file):
        print(f"Error: Could not find {save_file}")
        return

    print(f"Loading poses from {save_file}...")
    with open(save_file, "r") as f:
        recorded_poses = json.load(f)

    print(f"Loaded {len(recorded_poses)} poses successfully.")

    robot_ip = "192.168.50.110"
    print(f"\nConnecting to UR10 at {robot_ip}...")
    # try:
    #     robot = UrRtde(robot_ip)
    # except Exception as e:
    #     print(f"Failed to connect: {e}")
    #     return

    print("Successfully connected to the robot!")

    rospy.init_node('play_saved_poses', anonymous=True)

    vel = 0.5
    acc = 1.0

    record_proc = None

    print("\nWARNING: The robot will now move through the loaded points.")
    input("Stand clear and press Enter to execute (or Ctrl+C to cancel)...")

    print("\nPlaying back trajectory...")
    try:
        for i, pose in enumerate(recorded_poses):
            print(f"Moving to Pose {i+1}...")

            # Start recording before pose 2
            if i == 1 and record_proc is None:
                record_proc = start_event_recording(bag_file)
                time.sleep(1.0)  # give rosbag a moment to start

            # robot.move_TCP(pose, vel, acc)

            # Stop recording after pose 3
            if i == 2 and record_proc is not None:
                stop_event_recording(record_proc)
                record_proc = None

            time.sleep(0.5)

        print("\nPlayback complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        if record_proc is not None:
            stop_event_recording(record_proc)

if __name__ == "__main__":
    main()