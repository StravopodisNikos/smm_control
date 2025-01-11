#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <string>

class ForceTorquePublisher {
public:
    ForceTorquePublisher(ros::NodeHandle& nh) {
        // Load parameters from YAML
        if (!loadParameters(nh)) {
            ROS_ERROR("[ForceTorquePublisher] Failed to load parameters.");
            ros::shutdown();
        }

        // Initialize the publisher
        wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("/force_torque_wrench", 10);

        // Start publishing at a fixed rate
        ros::Rate loop_rate(publish_rate_);
        while (ros::ok()) {
            publishWrench();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    std::string link_name_;
    geometry_msgs::Wrench wrench_;
    ros::Publisher wrench_pub_;
    double publish_rate_;

    // Load parameters from the YAML file
    bool loadParameters(ros::NodeHandle& nh) {
        // Get the target link name
        if (!nh.getParam("/link_name", link_name_)) {
            ROS_ERROR("[ForceTorquePublisher] Missing parameter: link_name");
            return false;
        }
        // Get the wrench parameters
        std::vector<double> force, torque;
        if (!nh.getParam("/wrench/force", force) || force.size() != 3) {
            ROS_ERROR("[ForceTorquePublisher] Missing or invalid parameter: wrench/force");
            return false;
        }
        if (!nh.getParam("/wrench/torque", torque) || torque.size() != 3) {
            ROS_ERROR("[ForceTorquePublisher] Missing or invalid parameter: wrench/torque");
            return false;
        }

        // Set the wrench values
        wrench_.force.x = force[0];
        wrench_.force.y = force[1];
        wrench_.force.z = force[2];
        wrench_.torque.x = torque[0];
        wrench_.torque.y = torque[1];
        wrench_.torque.z = torque[2];

        // Get the publish rate
        if (!nh.getParam("publish_rate", publish_rate_)) {
            ROS_WARN("[ForceTorquePublisher] Missing parameter: publish_rate. Using default value 10 Hz.");
            publish_rate_ = 10.0;
        }

        return true;
    }

    // Publish the wrench message
    void publishWrench() {
        geometry_msgs::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.header.frame_id = link_name_;  // Target link frame
        wrench_msg.wrench = wrench_;
        wrench_pub_.publish(wrench_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ext_force_torque_publisher");
    ros::NodeHandle nh;
    ForceTorquePublisher force_torque_publisher(nh);

    return 0;
}
