import rosbag
import cv2
import numpy as np

bag_path = 'recordings/events_recording_001_start-20260430_205524_end-20260430_205540.bag'

def play_events():
    print("Playing events... Press 'q' to stop.")
    with rosbag.Bag(bag_path, "r") as bag:
        cv2.namedWindow("Event Camera", cv2.WINDOW_NORMAL)
        width, height = 346, 260
        frame = np.full((height, width, 3), 128, dtype=np.uint8)
        
        msg_count = 0
        for topic, msg, t in bag.read_messages(topics=['/capture_node/events']):
            if msg.width > 0 and msg.height > 0:
                if width != msg.width or height != msg.height:
                    width, height = msg.width, msg.height
                    frame = np.full((height, width, 3), 128, dtype=np.uint8)
                    
            for e in msg.events:
                val = 255 if e.polarity else 0
                if e.y < height and e.x < width:
                    frame[e.y, e.x] = (val, val, val)
                    
            msg_count += 1
            if msg_count >= 100: # approx 55 fps
                cv2.imshow("Event Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                frame.fill(128)
                msg_count = 0
                
    cv2.destroyAllWindows()

play_events()
