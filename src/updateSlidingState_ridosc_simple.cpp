#include <ros/ros.h>
#include <smm_control/IdoscError.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include "smm_control/timing.h"

// Global variables
std::vector<double> lambda_0(3, 0.01);  // Default values, overwritten by YAML
ros::Publisher sliding_surface_pub;     // Publisher for the sliding surface

// Callback for /idosc_error_state
void errorStateCallback(const smm_control::IdoscError::ConstPtr& msg) {
    // Check that the message contains valid data
    if (!msg) {
        ROS_ERROR("Received a null IdoscError message.");
        return;
    }

    // Calculate s = velocity_error + lambda_0 * position_error
    geometry_msgs::Vector3 s_msg;
    s_msg.x = msg->twist_error.linear.x + lambda_0[0] * msg->pose_error.position.x;
    s_msg.y = msg->twist_error.linear.y + lambda_0[1] * msg->pose_error.position.y;
    s_msg.z = msg->twist_error.linear.z + lambda_0[2] * msg->pose_error.position.z;

    // Publish sliding surface message
    sliding_surface_pub.publish(s_msg);

    // Debug output
    //ROS_INFO_STREAM("Sliding Surface s: [" << s_msg.x << ", " << s_msg.y << ", " << s_msg.z << "]");
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
    ros::init(argc, argv, "updateSlidingState_ridosc_simple");
    ros::NodeHandle nh;

    // Load lambda_0 parameters from the YAML file
    if (!loadParameters(nh)) {
        ROS_ERROR("Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscriber for the error state
    ros::Subscriber error_state_sub = nh.subscribe("/ridosc_error_state", 10, errorStateCallback);

    // Publisher for the sliding surface
    sliding_surface_pub = nh.advertise<geometry_msgs::Vector3>("/ridosc_sliding_surface", 10);

    ros::Rate loop_rate(UPDATE_SLIDING_SURFACE_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();  // Sleep according to the defined rate
    }
    return 0;
}
