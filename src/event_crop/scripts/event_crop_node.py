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

        rospy.loginfo(f"Subscribing to {input_topic}")
        rospy.loginfo(f"Publishing cropped events to {output_topic}")
        rospy.loginfo(f"Cropping to x:[{self.x_min},{self.x_max}) y:[{self.y_min},{self.y_max})")

        self.pub = rospy.Publisher(output_topic, EventArray, queue_size=10)
        rospy.Subscriber(input_topic, EventArray, self.callback, queue_size=10)
        rospy.spin()

    def callback(self, msg):
        cropped = EventArray()
        cropped.header = msg.header
        cropped.width  = self.x_max - self.x_min
        cropped.height = self.y_max - self.y_min

        events = [
            e for e in msg.events
            if self.x_min <= e.x < self.x_max
            and self.y_min <= e.y < self.y_max
        ]

        if self.shift_coordinates:
            cropped.events = [
                Event(x=e.x - self.x_min, y=e.y - self.y_min, ts=e.ts, polarity=e.polarity)
                for e in events
            ]
        else:
            cropped.events = events

        if cropped.events:
            self.pub.publish(cropped)

if __name__ == '__main__':
    EventCropNode()
