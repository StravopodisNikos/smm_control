#include <ros/ros.h>
#include <smm_control/FasmcError.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include "smm_control/timing.h"

// Global variables
std::vector<double> lambda_0(3, 0.01);  // Default values, overwritten by YAML
ros::Publisher sliding_surface_pub;     // Publisher for the sliding surface

// Callback for /fasmc_error_state
void errorStateCallback(const smm_control::FasmcError::ConstPtr& msg) {
    if (msg->position_error.size() != 3 || msg->velocity_error.size() != 3) {
        ROS_ERROR("Expected 3 values for position and velocity errors.");
        return;
    }

    // Calculate s = velocity_error + lambda_0 * position_error
    geometry_msgs::Vector3 s_msg;
    s_msg.x = msg->velocity_error[0] + lambda_0[0] * msg->position_error[0];
    s_msg.y = msg->velocity_error[1] + lambda_0[1] * msg->position_error[1];
    s_msg.z = msg->velocity_error[2] + lambda_0[2] * msg->position_error[2];

    // Publish sliding surface message
    sliding_surface_pub.publish(s_msg);

    // Debug output
    ROS_INFO_STREAM("Sliding Surface s: [" << s_msg.x << ", " << s_msg.y << ", " << s_msg.z << "]");
}

// Function to load parameters from the YAML file
bool loadParameters(ros::NodeHandle& nh) {
    if (nh.getParam("/lambda_0", lambda_0)) {
        if (lambda_0.size() != 3) {
            ROS_ERROR("lambda_0 should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded lambda_0 from parameter server.");
    } else {
        ROS_ERROR("Failed to load lambda_0 from parameter server.");
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateSlidingState_fasmc_simple");
    ros::NodeHandle nh;

    // Load lambda_0 parameters from the YAML file
    if (!loadParameters(nh)) {
        ROS_ERROR("Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscriber for the error state
    ros::Subscriber error_state_sub = nh.subscribe("/fasmc_error_state", 10, errorStateCallback);

    // Publisher for the sliding surface
    sliding_surface_pub = nh.advertise<geometry_msgs::Vector3>("/fasmc_sliding_surface", 10);

    ros::Rate loop_rate(UPDATE_SLIDING_SURFACE_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();  // Sleep according to the defined rate
    }
    return 0;
}
