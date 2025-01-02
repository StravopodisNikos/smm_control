#include <ros/ros.h>
#include <smm_control/CustomTcpState.h>
#include <vector>
#include <string>
#include "smm_control/timing.h"

// Global variable to hold the single joint state message
smm_control::CustomTcpState fixed_tcp_state;
ros::Publisher desired_state_pub;

// Function to load the fixed state from the YAML file on the parameter server
bool loadParameters(ros::NodeHandle& nh) {
    std::vector<double> position_data, velocity_data, acceleration_data;
    ros::NodeHandle private_nh("~"); // Private NodeHandle for node-specific parameters in launch file. Fucking crazy.
    std::string tcp_link_name;

    if (!private_nh.getParam("TCP_LINK_NAME", tcp_link_name)) {
        ROS_ERROR("[publishSingleTcpGoal/loadParameters] Failed to load tcp_link_name parameter.");
        return false;
    }

    // Load position data
    if (!nh.getParam("/cartesian_positions/position", position_data) || position_data.size() != 3) {
        ROS_ERROR("[publishSingleTcpGoal/loadParameters] Failed to load position data or incorrect size.");
        return false;
    }

    // Load velocity data
    if (!nh.getParam("/cartesian_velocities/velocity", velocity_data) || velocity_data.size() != 3) {
        ROS_ERROR("[publishSingleTcpGoal/loadParameters] Failed to load velocity data or incorrect size.");
        return false;
    }

    // Load acceleration data
    if (!nh.getParam("/cartesian_accelerations/acceleration", acceleration_data) || acceleration_data.size() != 3) {
        ROS_ERROR("[publishSingleTcpGoal/loadParameters] Failed to load acceleration data or incorrect size.");
        return false;
    }

    // Populate the CustomJointState message
    fixed_tcp_state.tcp_link_name = tcp_link_name;
    fixed_tcp_state.position = {position_data[0], position_data[1], position_data[2]};
    fixed_tcp_state.velocity = {velocity_data[0], velocity_data[1], velocity_data[2]};
    fixed_tcp_state.acceleration = {acceleration_data[0], acceleration_data[1], acceleration_data[2]};

    return true;
}

// Function to publish the fixed desired state
void publishDesiredState() {
    desired_state_pub.publish(fixed_tcp_state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "publishSingleTcpGoal");
    ros::NodeHandle nh;

    // Load parameters
    if (!loadParameters(nh)) {
        ROS_ERROR("[publishSingleTcpGoal] Failed to load TCP trajectory data.");
        return -1;
    }

    // Initialize the publisher for /tcp_desired_state
    desired_state_pub = nh.advertise<smm_control::CustomTcpState>("/tcp_desired_state", 10);

    ros::Rate loop_rate(UPDATE_JOINT_PATH_LOOP_RATE);

    while (ros::ok()) {
        publishDesiredState();  // Publish the fixed desired state
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
