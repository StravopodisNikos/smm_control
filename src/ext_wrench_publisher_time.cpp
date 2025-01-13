#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <string>

class ForceTorquePublisher {
public:
    ForceTorquePublisher(ros::NodeHandle& nh, double start_delay, double publish_duration)
        : start_delay_(start_delay), publish_duration_(publish_duration) {
        // Load parameters from YAML
        if (!loadParameters(nh)) {
            ROS_ERROR("[ForceTorquePublisher] Failed to load parameters.");
            ros::shutdown();
        }

        // Initialize the publisher
        wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("/force_torque_wrench", 10);

        // Start publishing at a fixed rate
        ros::Rate loop_rate(publish_rate_);
        ros::Time start_time = ros::Time::now();

        bool publish_force = false;

        while (ros::ok()) {
            double elapsed_time = (ros::Time::now() - start_time).toSec();

            if (elapsed_time < start_delay_) {
                // Publish zero wrench until start delay is reached
                publishZeroWrench();
            } else if (elapsed_time < (start_delay_ + publish_duration_)) {
                // Publish the specified wrench during the desired time window
                publishWrench();
                publish_force = true;  // Indicate that the force wrench was applied
            } else {
                // After the duration, publish zero wrench indefinitely
                publishZeroWrench();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    std::string link_name_;
    geometry_msgs::Wrench wrench_;
    ros::Publisher wrench_pub_;
    double publish_rate_;
    double start_delay_;
    double publish_duration_;

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
            publish_rate_ = 1.0;
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

    // Publish a zero wrench to stop applying forces
    void publishZeroWrench() {
        geometry_msgs::WrenchStamped zero_wrench_msg;
        zero_wrench_msg.header.stamp = ros::Time::now();
        zero_wrench_msg.header.frame_id = link_name_;  // Target link frame

        // Set very small force and zero torque
        zero_wrench_msg.wrench.force.x = 0.001;
        zero_wrench_msg.wrench.force.y = 0.001;
        zero_wrench_msg.wrench.force.z = 0.001;
        zero_wrench_msg.wrench.torque.x = 0.0;
        zero_wrench_msg.wrench.torque.y = 0.0;
        zero_wrench_msg.wrench.torque.z = 0.0;

        wrench_pub_.publish(zero_wrench_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ext_wrench_publisher_time");
    ros::NodeHandle nh;

    // Retrieve arguments for start delay and publish duration
    if (argc < 3) {
        ROS_ERROR("Usage: ext_wrench_publisher_time <start_delay> <publish_duration>");
        return -1;
    }

    double start_delay = std::stod(argv[1]);
    double publish_duration = std::stod(argv[2]);

    ForceTorquePublisher force_torque_publisher(nh, start_delay, publish_duration);

    return 0;
}
