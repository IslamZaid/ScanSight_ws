/*
 * C++ event cropping node — near-zero overhead.
 * Subscribes to EventArray, filters to ROI, publishes cropped events.
 */
#include <ros/ros.h>
#include <dv_ros_msgs/EventArray.h>

class EventCropNode {
public:
    EventCropNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        pnh.param("x_min", x_min_, 0);
        pnh.param("x_max", x_max_, 640);
        pnh.param("y_min", y_min_, 0);
        pnh.param("y_max", y_max_, 480);
        pnh.param("shift_coordinates", shift_, true);

        std::string input_topic, output_topic;
        pnh.param<std::string>("input_topic",  input_topic,  "/capture_node/events");
        pnh.param<std::string>("output_topic", output_topic, "/event_crop/events");

        int crop_w = x_max_ - x_min_;
        int crop_h = y_max_ - y_min_;
        ROS_INFO("Crop: x=[%d,%d) y=[%d,%d) -> %dx%d",
                 x_min_, x_max_, y_min_, y_max_, crop_w, crop_h);

        pub_ = nh.advertise<dv_ros_msgs::EventArray>(output_topic, 50);
        sub_ = nh.subscribe(input_topic, 50, &EventCropNode::callback, this);
    }

private:
    void callback(const dv_ros_msgs::EventArray::ConstPtr& msg) {
        if (msg->events.empty()) return;

        dv_ros_msgs::EventArray cropped;
        cropped.header = msg->header;
        cropped.width  = x_max_ - x_min_;
        cropped.height = y_max_ - y_min_;

        cropped.events.reserve(msg->events.size());

        for (const auto& e : msg->events) {
            if (e.x >= x_min_ && e.x < x_max_ &&
                e.y >= y_min_ && e.y < y_max_) {
                dv_ros_msgs::Event ce;
                if (shift_) {
                    ce.x = e.x - x_min_;
                    ce.y = e.y - y_min_;
                } else {
                    ce.x = e.x;
                    ce.y = e.y;
                }
                ce.ts = e.ts;
                ce.polarity = e.polarity;
                cropped.events.push_back(ce);
            }
        }

        if (!cropped.events.empty()) {
            pub_.publish(cropped);
        }
    }

    ros::Subscriber sub_;
    ros::Publisher  pub_;
    int x_min_, x_max_, y_min_, y_max_;
    bool shift_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "event_crop_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    EventCropNode node(nh, pnh);
    ros::spin();
    return 0;
}
