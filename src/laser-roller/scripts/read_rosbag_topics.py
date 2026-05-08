#!/usr/bin/python3
import argparse
import glob
import os
import sys
from datetime import datetime
import numpy as np

import rosbag



def default_recordings_dir():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, "recordings")


def find_latest_bag(recordings_dir):
    bag_files = glob.glob(os.path.join(recordings_dir, "*.bag"))
    if not bag_files:
        return None

    return max(bag_files, key=os.path.getmtime)


def format_bag_time(timestamp):
    return datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")


def print_bag_topics(bag_path):
    if not os.path.exists(bag_path):
        print(f"Error: bag file not found: {bag_path}")
        return 1

    with rosbag.Bag(bag_path, "r") as bag:
        info = bag.get_type_and_topic_info()
        topics = info.topics

        print(f"Bag file: {bag_path}")
        print(f"Size: {os.path.getsize(bag_path)} bytes")

        try:
            print(f"Start: {format_bag_time(bag.get_start_time())}")
            print(f"End:   {format_bag_time(bag.get_end_time())}")
            print(f"Duration: {bag.get_end_time() - bag.get_start_time():.3f} seconds")
        except rosbag.ROSBagException:
            print("Start: no messages")
            print("End:   no messages")
            print("Duration: 0.000 seconds")

        if not topics:
            print("\nNo topics found in this bag.")
            return 0

        print("\nTopics:")
        for topic_name in sorted(topics):
            topic_info = topics[topic_name]
            frequency = topic_info.frequency
            frequency_text = "unknown" if frequency is None else f"{frequency:.2f} Hz"
            print(f"- {topic_name}")
            print(f"  type: {topic_info.msg_type}")
            print(f"  messages: {topic_info.message_count}")
            print(f"  frequency: {frequency_text}")

    return 0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Print the topics stored inside a ROS bag file."
    )
    parser.add_argument(
        "bag",
        nargs="?",
        help="Path to a .bag file. If omitted, the newest bag in scripts/recordings is used.",
    )
    parser.add_argument(
        "--play",
        action="store_true",
        help="Play the event camera recording to /capture_node/events/image for rqt_image_view",
    )
    return parser.parse_args()

def play_events_to_ros(bag_path):
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rospy.init_node('play_events_node', anonymous=True)
    pub = rospy.Publisher('/capture_node/events/image', Image, queue_size=10)
    bridge = CvBridge()

    print(f"\nPublishing events from {bag_path} to /capture_node/events/image")
    print("Open 'rqt_image_view' and select '/capture_node/events/image' to see the stream.")

    with rosbag.Bag(bag_path, "r") as bag:
        width, height = 346, 260
        frame = np.full((height, width, 3), 128, dtype=np.uint8)
        
        frame_start_time = None
        frame_duration = 1.0 / 30.0  # 30 FPS
        
        real_start_time = rospy.Time.now().to_sec()
        bag_start_time = None

        try:
            for topic, msg, t in bag.read_messages(topics=['/capture_node/events']):
                if rospy.is_shutdown():
                    break

                current_t = t.to_sec()
                if bag_start_time is None:
                    bag_start_time = current_t
                    real_start_time = rospy.Time.now().to_sec()
                if frame_start_time is None:
                    frame_start_time = current_t

                if msg.width > 0 and msg.height > 0:
                    if width != msg.width or height != msg.height:
                        width, height = msg.width, msg.height
                        frame = np.full((height, width, 3), 128, dtype=np.uint8)
                        
                if msg.events:
                    # Extremely fast extraction using numpy
                    xs = np.fromiter((e.x for e in msg.events), dtype=np.uint16)
                    ys = np.fromiter((e.y for e in msg.events), dtype=np.uint16)
                    pols = np.fromiter((e.polarity for e in msg.events), dtype=bool)

                    # Filter bounds safely
                    valid = (xs < width) & (ys < height)
                    if not valid.all():
                        xs = xs[valid]
                        ys = ys[valid]
                        pols = pols[valid]

                    # Vectorized assignment
                    frame[ys[pols], xs[pols]] = 255
                    frame[ys[~pols], xs[~pols]] = 0
                        
                if current_t - frame_start_time >= frame_duration:
                    img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    if hasattr(msg, 'header'):
                        img_msg.header = msg.header
                    pub.publish(img_msg)
                    
                    frame.fill(128)
                    frame_start_time = current_t
                    
                    # Sync with real-time
                    elapsed_bag = current_t - bag_start_time
                    elapsed_real = rospy.Time.now().to_sec() - real_start_time
                    if elapsed_bag > elapsed_real:
                        rospy.sleep(elapsed_bag - elapsed_real)
            print("Finished playback.")
        except KeyboardInterrupt:
            print("Playback interrupted.")


def main():
    args = parse_args()
    bag_path = args.bag

    if bag_path is None:
        recordings_dir = default_recordings_dir()
        bag_path = find_latest_bag(recordings_dir)
        if bag_path is None:
            print(f"Error: no .bag files found in {recordings_dir}")
            return 1

    bag_path = os.path.abspath(bag_path)
    print_bag_topics(bag_path)

    if args.play:
        play_events_to_ros(bag_path)

    return 0


if __name__ == "__main__":
    sys.exit(main())
