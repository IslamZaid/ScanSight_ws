#!/usr/bin/python3
import time
import json
import os
import rospy
from ur_rtde import UrRtde

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    save_file = os.path.join(script_dir, "saved_poses.json")
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
    
    # Initialize ROS node to use rospy parameters
    rospy.init_node('play_saved_poses', anonymous=True)

    vel = 0.5  # m/s
    
    acc = 1.0  # m/s^2

    print("\nWARNING: The robot will now move through the loaded points.")
    input("Stand clear and press Enter to execute (or Ctrl+C to cancel)...")


    print("\nPlaying back trajectory...")
    for i, pose in enumerate(recorded_poses):
        print(f"Moving to Pose {i+1}...")
        if i == 1:  
            rospy.set_param('/rosbag_name', "test_laser_recording")
            rospy.set_param('/recording_enabled', True)
            rospy.sleep(0.5)

        robot.move_TCP(pose, vel, acc)
        if i == 2:
            rospy.set_param('/recording_enabled', False)
            rospy.sleep(1.0)
            
        time.sleep(0.5) # Short pause between points
        
    print("\nPlayback complete!")

if __name__ == "__main__":
    main()
