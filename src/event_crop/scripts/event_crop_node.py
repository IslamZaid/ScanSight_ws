#!/usr/bin/env python3
import rospy
from dv_ros_msgs.msg import EventArray, Event

class EventCropNode:
    def __init__(self):
        rospy.init_node('event_crop_node')

        self.x_min = rospy.get_param('~x_min', 0)
        self.x_max = rospy.get_param('~x_max', 100)
        self.y_min = rospy.get_param('~y_min', 0)
        self.y_max = rospy.get_param('~y_max', 100)

        input_topic  = rospy.get_param('~input_topic',  '/capture_node/events')
        output_topic = rospy.get_param('~output_topic', '/event_crop/events')
        self.shift_coordinates = rospy.get_param('~shift_coordinates', True)
        self.rotate_90 = rospy.get_param('~rotate_90', False)

        rospy.loginfo(f"Subscribing to {input_topic}")
        rospy.loginfo(f"Publishing cropped events to {output_topic}")
        rospy.loginfo(f"Cropping to x:[{self.x_min},{self.x_max}) y:[{self.y_min},{self.y_max})")
        if self.rotate_90:
            rospy.loginfo("Rotating output 90° CCW (swapping x/y)")

        self.pub = rospy.Publisher(output_topic, EventArray, queue_size=10)
        rospy.Subscriber(input_topic, EventArray, self.callback, queue_size=10)
        rospy.spin()

    def callback(self, msg):
        crop_w = self.x_max - self.x_min
        crop_h = self.y_max - self.y_min

        cropped = EventArray()
        cropped.header = msg.header

        events = [
            e for e in msg.events
            if self.x_min <= e.x < self.x_max
            and self.y_min <= e.y < self.y_max
        ]

        if self.shift_coordinates and self.rotate_90:
            # Shift then rotate 90° CCW: new_x = y, new_y = (crop_w - 1) - x
            cropped.width  = crop_h
            cropped.height = crop_w
            cropped.events = [
                Event(x=e.y - self.y_min,
                      y=(crop_w - 1) - (e.x - self.x_min),
                      ts=e.ts, polarity=e.polarity)
                for e in events
            ]
        elif self.shift_coordinates:
            cropped.width  = crop_w
            cropped.height = crop_h
            cropped.events = [
                Event(x=e.x - self.x_min, y=e.y - self.y_min, ts=e.ts, polarity=e.polarity)
                for e in events
            ]
        elif self.rotate_90:
            cropped.width  = crop_h
            cropped.height = crop_w
            cropped.events = [
                Event(x=e.y, y=(crop_w - 1) - e.x, ts=e.ts, polarity=e.polarity)
                for e in events
            ]
        else:
            cropped.width  = crop_w
            cropped.height = crop_h
            cropped.events = events

        if cropped.events:
            self.pub.publish(cropped)

if __name__ == '__main__':
    EventCropNode()
