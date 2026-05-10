#include <ros/ros.h>
#include <dv_ros_messaging/messaging.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>
#include <dv-processing/noise/fast_decay_noise_filter.hpp>
#include <dv-processing/noise/k_noise_filter.hpp>

class EventFilterNode {
public:
    EventFilterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        pnh.param("time_window_us", time_window_us_, 10000);
        pnh.param<std::string>("filter_type", filter_type_, "k_noise");

        std::string in_topic, out_topic;
        pnh.param<std::string>("input_topic",  in_topic,  "/laser_event_processing/events_cropped");
        pnh.param<std::string>("output_topic", out_topic, "/laser_event_processing/events_filtered");

        pub_ = nh.advertise<dv_ros_msgs::EventArrayMessage>(out_topic, 50);
        sub_ = nh.subscribe<dv_ros_msgs::EventArrayMessage>(
            in_topic, 50, &EventFilterNode::callback, this);

        ROS_INFO("Filter type: %s, window: %d us — waiting for first message to read dimensions",
                 filter_type_.c_str(), time_window_us_);
    }

private:
    void initFilter(int width, int height) {
        cv::Size resolution(width, height);
        if (filter_type_ == "fast_decay") {
            fast_decay_ = std::make_unique<dv::noise::FastDecayNoiseFilter<>>(
                resolution, dv::Duration(time_window_us_));
        } else if (filter_type_ == "ba") {
            ba_ = std::make_unique<dv::noise::BackgroundActivityNoiseFilter<>>(
                resolution, dv::Duration(time_window_us_));
        } else {
            k_noise_ = std::make_unique<dv::noise::KNoiseFilter<>>(
                resolution, dv::Duration(time_window_us_));
        }
        width_  = width;
        height_ = height;
        ROS_INFO("Filter initialised: %s, %dx%d, window=%d us",
                 filter_type_.c_str(), width_, height_, time_window_us_);
    }

    void callback(const dv_ros_msgs::EventArrayMessage::ConstPtr& msg) {
        if (msg->events.empty()) return;

        int w = static_cast<int>(msg->width);
        int h = static_cast<int>(msg->height);

        if (!ba_ && !fast_decay_ && !k_noise_ || w != width_ || h != height_) {
            initFilter(w, h);
        }

        dv::EventStore store = dv_ros_msgs::toEventStore(*msg);

        dv::EventStore filtered;
        if (filter_type_ == "fast_decay") {
            fast_decay_->accept(store);
            filtered = fast_decay_->generateEvents();
        } else if (filter_type_ == "ba") {
            ba_->accept(store);
            filtered = ba_->generateEvents();
        } else {
            k_noise_->accept(store);
            filtered = k_noise_->generateEvents();
        }

        if (filtered.isEmpty()) return;

        if (++callback_count_ % 100 == 0) {
            double r = filter_type_ == "fast_decay" ? fast_decay_->getReductionFactor()
                     : filter_type_ == "ba"         ? ba_->getReductionFactor()
                                                    : k_noise_->getReductionFactor();
            ROS_INFO_THROTTLE(5.0, "Filter reduction: %.1f%% of events removed as noise", r * 100.0);
        }

        auto out = dv_ros_msgs::toRosEventsMessage(filtered, cv::Size(width_, height_));
        pub_.publish(out);
    }

    ros::Subscriber sub_;
    ros::Publisher  pub_;

    int time_window_us_;
    int width_  = 0;
    int height_ = 0;
    int callback_count_ = 0;
    std::string filter_type_;

    std::unique_ptr<dv::noise::BackgroundActivityNoiseFilter<>> ba_;
    std::unique_ptr<dv::noise::FastDecayNoiseFilter<>>          fast_decay_;
    std::unique_ptr<dv::noise::KNoiseFilter<>>                  k_noise_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "event_filter_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    EventFilterNode node(nh, pnh);
    ros::spin();
    return 0;
}
