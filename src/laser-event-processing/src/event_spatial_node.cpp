#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <dv_ros_msgs/EventArray.h>
#include <laser_event_processing/EventSpatialArray.h>

#include <deque>
#include <cmath>
#include <mutex>

struct VelSample {
    double stamp;  // ROS time in seconds
    float  v;      // linear speed magnitude (m/s)
};

class EventSpatialNode {
public:
    EventSpatialNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        std::string in_topic, out_topic, vel_topic;
        pnh.param<std::string>("input_topic",    in_topic,  "/laser_event_processing/events_filtered");
        pnh.param<std::string>("output_topic",   out_topic, "/laser_event_processing/events_spatial");
        pnh.param<std::string>("velocity_topic", vel_topic, "/laser_roller/tcp_velocity");
        pnh.param("vel_buffer_sec", vel_buffer_sec_, 2.0);

        pub_     = nh.advertise<laser_event_processing::EventSpatialArray>(out_topic, 50);
        sub_vel_ = nh.subscribe(vel_topic, 500, &EventSpatialNode::velCallback, this);
        sub_evt_ = nh.subscribe(in_topic,  50,  &EventSpatialNode::evtCallback, this);

        ROS_INFO("event_spatial_node ready");
        ROS_INFO("  events in:  %s", in_topic.c_str());
        ROS_INFO("  velocity:   %s", vel_topic.c_str());
        ROS_INFO("  spatial out:%s", out_topic.c_str());
        ROS_INFO("  z = sum(dt_i * v_i)  [meters, cumulative]");
    }

private:
    // ---------------------------------------------------------------------------
    //  Velocity buffer
    // ---------------------------------------------------------------------------
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        float vx = msg->twist.linear.x;
        float vy = msg->twist.linear.y;
        float vz = msg->twist.linear.z;
        float v  = std::sqrt(vx*vx + vy*vy + vz*vz);

        bool negate = false;
        ros::param::get("~negate_velocity", negate);
        if (negate) v = -v;

        std::lock_guard<std::mutex> lock(vel_mutex_);
        vel_buf_.push_back({msg->header.stamp.toSec(), v});

        double cutoff = msg->header.stamp.toSec() - vel_buffer_sec_;
        while (!vel_buf_.empty() && vel_buf_.front().stamp < cutoff)
            vel_buf_.pop_front();
    }

    // Linear interpolation between the two bracketing velocity samples.
    float interpolateVelocity(double t) {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        if (vel_buf_.empty()) return 0.0f;

        const VelSample* prev = &vel_buf_.front();
        for (const auto& s : vel_buf_) {
            if (s.stamp <= t) {
                prev = &s;
            } else {
                double dt = s.stamp - prev->stamp;
                if (dt < 1e-9) return s.v;
                double alpha = (t - prev->stamp) / dt;
                return static_cast<float>(prev->v + alpha * (s.v - prev->v));
            }
        }
        return vel_buf_.back().v;
    }

    // ---------------------------------------------------------------------------
    //  Event callback — assign z = cumulative displacement
    // ---------------------------------------------------------------------------
    void evtCallback(const dv_ros_msgs::EventArray::ConstPtr& msg) {
        if (msg->events.empty()) return;

        laser_event_processing::EventSpatialArray out;
        out.header = msg->header;
        out.width  = msg->width;
        out.height = msg->height;
        out.events.reserve(msg->events.size());

        for (const auto& e : msg->events) {
            double t = e.ts.toSec();

            // initialise previous timestamp on first event ever seen
            if (!prev_t_set_) {
                prev_t_   = t;
                prev_t_set_ = true;
            }

            double dt = t - prev_t_;

            // guard against out-of-order or identical timestamps
            if (dt < 0.0) dt = 0.0;

            float v = interpolateVelocity(t);

            // z = cumulative distance traveled (meters)
            z_accum_ += static_cast<float>(dt * v);
            prev_t_   = t;

            laser_event_processing::EventSpatial se;
            se.x        = e.x;
            se.y        = e.y;
            se.z        = z_accum_;
            se.polarity = e.polarity;
            out.events.push_back(se);
        }

        pub_.publish(out);
    }

    ros::Subscriber sub_evt_;
    ros::Subscriber sub_vel_;
    ros::Publisher  pub_;

    std::mutex            vel_mutex_;
    std::deque<VelSample> vel_buf_;
    double                vel_buffer_sec_;

    double prev_t_     = 0.0;
    bool   prev_t_set_ = false;
    float  z_accum_    = 0.0f;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "event_spatial_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    EventSpatialNode node(nh, pnh);
    ros::spin();
    return 0;
}
