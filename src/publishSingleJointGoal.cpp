#include <ros/ros.h>
#include <smm_control/CustomJointState.h>
#include <vector>
#include <string>
#include "smm_control/timing.h"

// Global variable to hold the single joint state message
smm_control::CustomJointState fixed_joint_state;
ros::Publisher desired_state_pub;

// Function to load the fixed state from the YAML file on the parameter server
bool loadParameters(ros::NodeHandle& nh) {
    std::vector<double> position_data, velocity_data, acceleration_data;

    // Load position data
    if (!nh.getParam("/positions/position", position_data) || position_data.size() != 3) {
        ROS_ERROR("[publishSingleJointGoal/loadParameters] Failed to load position data or incorrect size.");
        return false;
    }

    // Load velocity data
    if (!nh.getParam("/velocities/velocity", velocity_data) || velocity_data.size() != 3) {
        ROS_ERROR("[publishSingleJointGoal/loadParameters] Failed to load velocity data or incorrect size.");
        return false;
    }

    // Load acceleration data
    if (!nh.getParam("/accelerations/acceleration", acceleration_data) || acceleration_data.size() != 3) {
        ROS_ERROR("[publishSingleJointGoal/loadParameters] Failed to load acceleration data or incorrect size.");
        return false;
    }

    // Populate the CustomJointState message
    fixed_joint_state.name = {"joint1", "joint2", "joint3"};
    fixed_joint_state.position = {position_data[0], position_data[1], position_data[2]};
    fixed_joint_state.velocity = {velocity_data[0], velocity_data[1], velocity_data[2]};
    fixed_joint_state.acceleration = {acceleration_data[0], acceleration_data[1], acceleration_data[2]};

    return true;
}

// Function to publish the fixed desired state
void publishDesiredState() {
    desired_state_pub.publish(fixed_joint_state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "publishSingleJointGoal");
    ros::NodeHandle nh;

    // Load parameters
    if (!loadParameters(nh)) {
        ROS_ERROR("[publishSingleJointGoal] Failed to load joint trajectory data.");
        return -1;
    }

    // Initialize the publisher for /joint_desired_state
    desired_state_pub = nh.advertise<smm_control::CustomJointState>("/joint_desired_state", 10);

    ros::Rate loop_rate(UPDATE_JOINT_PATH_LOOP_RATE);

    while (ros::ok()) {
        publishDesiredState();  // Publish the fixed desired state
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
