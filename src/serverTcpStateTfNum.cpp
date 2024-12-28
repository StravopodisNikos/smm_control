#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <smm_control/GetTcpStateTfNum.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>

class TCPVelAccelService {
public:
    TCPVelAccelService() : tf_listener_(tf_buffer_), prev_time_(0.0), alpha_(0.8) {
        // Load parameters
        if (!nh_.getParam("base_frame", base_frame_)) {
            ROS_ERROR("Failed to get 'base_frame' parameter.");
            ros::shutdown();
        }

        if (!nh_.getParam("tcp_frame", tcp_frame_)) {
            ROS_ERROR("Failed to get 'tcp_frame' parameter.");
            ros::shutdown();
        }

        // Advertise the service
        service_ = nh_.advertiseService("GetTcpVelAccelNum", &TCPVelAccelService::computeVelAccel, this);
    }

    bool computeVelAccel(smm_control::GetTcpStateTfNum::Request &req,
                         smm_control::GetTcpStateTfNum::Response &res) {
        // Get current time
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - prev_time_).toSec();

        if (dt == 0) {
            ROS_WARN("Time step is zero, skipping computation.");
            return false;
        }

        // Lookup transform
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(base_frame_, tcp_frame_, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }

        // Extract current position
        Eigen::Vector3d current_position(transform.transform.translation.x,
                                          transform.transform.translation.y,
                                          transform.transform.translation.z);

        // Compute velocity
        Eigen::Vector3d current_velocity = (current_position - prev_position_) / dt;

        // Apply complementary filter for velocity
        filtered_velocity_ = alpha_ * current_velocity + (1 - alpha_) * filtered_velocity_;

        // Compute acceleration
        Eigen::Vector3d current_acceleration = (filtered_velocity_ - prev_velocity_) / dt;

        // Apply complementary filter for acceleration
        filtered_acceleration_ = alpha_ * current_acceleration + (1 - alpha_) * filtered_acceleration_;

        // Update previous states
        prev_time_ = current_time;
        prev_position_ = current_position;
        prev_velocity_ = filtered_velocity_;

        // Populate service response
        if (req.get_vel_num) {
            res.vel_num = {static_cast<float>(filtered_velocity_.x()),
                           static_cast<float>(filtered_velocity_.y()),
                           static_cast<float>(filtered_velocity_.z())};
        }

        if (req.get_accel_num) {
            res.accel_num = {static_cast<float>(filtered_acceleration_.x()),
                             static_cast<float>(filtered_acceleration_.y()),
                             static_cast<float>(filtered_acceleration_.z())};
        }

        return true;
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::ServiceServer service_;

    std::string base_frame_;
    std::string tcp_frame_;

    ros::Time prev_time_;
    Eigen::Vector3d prev_position_;
    Eigen::Vector3d prev_velocity_;

    Eigen::Vector3d filtered_velocity_;
    Eigen::Vector3d filtered_acceleration_;

    double alpha_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "serverTcpStateTfNum");

    TCPVelAccelService service;

    ros::spin();
    return 0;
}
