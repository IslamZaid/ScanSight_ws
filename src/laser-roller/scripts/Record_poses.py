#!/usr/bin/python3
import time
import json
import os
from ur_rtde import UrRtde

def main():
    robot_ip = "192.168.50.110"

    print(f"Connecting to UR10 at {robot_ip}...")
    robot = None
    try:
        robot = UrRtde(robot_ip)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    try:
        print("\nSuccessfully connected to the robot!")
        print("==================================================")
        print(" TEACH MODE ")
        print("==================================================")
        print("1. Manually move the robot to a desired position using the teach pendant (Free-Drive).")
        print("2. Press ENTER to record its current pose.")
        print("3. Repeat for as many poses as you need (a, b, c, d, etc.).")
        print("4. Type 'q' and press ENTER when you are completely finished recording.")
        print("==================================================\n")

        recorded_poses = []

        # Loop to collect as many poses as the user wants
        while True:
            user_input = input(f"Move robot to Pose {len(recorded_poses) + 1} and press ENTER (or type 'q' to finish): ")

            if user_input.strip().lower() in ['q', 'quit', 'done', 'exit']:
                break

            current_pose = list(robot.get_pose())
            recorded_poses.append(current_pose)
            print(f"--> Saved Pose {len(recorded_poses)}: {current_pose}\n")

        if not recorded_poses:
            print("No poses were recorded. Exiting.")
            return

        print("\n==================================================")
        print(f" Successfully recorded {len(recorded_poses)} poses.")
        print("==================================================")

        # Save to file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        save_file = os.path.join(script_dir, "saved_poses.json")
        with open(save_file, "w") as f:
            json.dump(recorded_poses, f, indent=4)
        print(f"Poses have been permanently saved to: {os.path.abspath(save_file)}")

        # Ask if the user wants to play them back
        playback = input("Would you like the robot to automatically move through these poses now? (y/n): ")
        if playback.strip().lower() in ['y', 'yes']:
            vel = 0.2  # m/s
            acc = 0.2  # m/s^2

            print("\nWARNING: The robot will now move through your recorded points.")
            input("Stand clear and press Enter to execute (or Ctrl+C to cancel)...")

            print("\nPlaying back trajectory...")
            for i, pose in enumerate(recorded_poses):
                print(f"Moving to Pose {i + 1}...")
                robot.move_TCP(pose, vel, acc)
                time.sleep(0.5)  # Short pause between points

            print("\nPlayback complete!")
        else:
            print("Playback skipped. Exiting.")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        if robot is not None:
            print("Disconnecting from robot...")
            robot.disconnect()
            print("Disconnected.")

if __name__ == "__main__":
    main()
