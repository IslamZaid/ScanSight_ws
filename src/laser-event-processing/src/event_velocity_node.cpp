#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <dv_ros_msgs/EventArray.h>
#include <laser_event_processing/EventVArray.h>

#include <deque>
#include <cmath>
#include <mutex>

struct VelSample {
    double stamp;   // seconds
    float  v;       // total linear speed (m/s)
};

class EventVelocityNode {
public:
    EventVelocityNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        std::string in_topic, out_topic, vel_topic;
        pnh.param<std::string>("input_topic",    in_topic,  "/laser_event_processing/events_filtered");
        pnh.param<std::string>("output_topic",   out_topic, "/laser_event_processing/events_velocity");
        pnh.param<std::string>("velocity_topic", vel_topic, "/laser_roller/tcp_velocity");
        pnh.param("vel_buffer_sec", vel_buffer_sec_, 1.0);

        pub_     = nh.advertise<laser_event_processing::EventVArray>(out_topic, 50);
        sub_vel_ = nh.subscribe(vel_topic, 200, &EventVelocityNode::velCallback, this);
        sub_evt_ = nh.subscribe(in_topic,  50,  &EventVelocityNode::evtCallback, this);

        ROS_INFO("event_velocity_node ready");
        ROS_INFO("  events:   %s", in_topic.c_str());
        ROS_INFO("  velocity: %s", vel_topic.c_str());
        ROS_INFO("  output:   %s", out_topic.c_str());
    }

private:
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        float vx = msg->twist.linear.x;
        float vy = msg->twist.linear.y;
        float vz = msg->twist.linear.z;
        float v  = std::sqrt(vx*vx + vy*vy + vz*vz);

        std::lock_guard<std::mutex> lock(vel_mutex_);
        vel_buf_.push_back({msg->header.stamp.toSec(), v});

        // trim old samples
        double cutoff = msg->header.stamp.toSec() - vel_buffer_sec_;
        while (!vel_buf_.empty() && vel_buf_.front().stamp < cutoff)
            vel_buf_.pop_front();
    }

    float lookupVelocity(double event_stamp) {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        if (vel_buf_.empty()) return 0.0f;

        // find the two samples that bracket event_stamp and interpolate
        const VelSample* prev = &vel_buf_.front();
        for (const auto& s : vel_buf_) {
            if (s.stamp <= event_stamp) prev = &s;
            else {
                // interpolate between prev and s
                double dt = s.stamp - prev->stamp;
                if (dt < 1e-9) return s.v;
                double alpha = (event_stamp - prev->stamp) / dt;
                return static_cast<float>(prev->v + alpha * (s.v - prev->v));
            }
        }
        // event is after all samples — use latest
        return vel_buf_.back().v;
    }

    void evtCallback(const dv_ros_msgs::EventArray::ConstPtr& msg) {
        if (msg->events.empty()) return;

        laser_event_processing::EventVArray out;
        out.header = msg->header;
        out.width  = msg->width;
        out.height = msg->height;
        out.events.reserve(msg->events.size());

        for (const auto& e : msg->events) {
            double t = e.ts.toSec();
            float  v = lookupVelocity(t);

            laser_event_processing::EventV ev;
            ev.x        = e.x;
            ev.y        = e.y;
            ev.ts       = e.ts;
            ev.polarity = e.polarity;
            ev.velocity = v;
            out.events.push_back(ev);
        }

        pub_.publish(out);
    }

    ros::Subscriber sub_evt_;
    ros::Subscriber sub_vel_;
    ros::Publisher  pub_;

    std::mutex           vel_mutex_;
    std::deque<VelSample> vel_buf_;
    double               vel_buffer_sec_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "event_velocity_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    EventVelocityNode node(nh, pnh);
    ros::spin();
    return 0;
}
