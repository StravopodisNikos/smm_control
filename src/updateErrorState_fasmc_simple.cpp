#include <ros/ros.h>
#include <smm_control/CustomJointState.h>
#include <sensor_msgs/JointState.h>
#include <smm_control/FasmcError.h>  
#include <vector>
#include "smm_control/timing.h"

// Global variables to store the latest desired and current joint states
smm_control::CustomJointState latest_desired_state;
sensor_msgs::JointState latest_current_state;

// Flags to indicate if messages have been received
bool desired_state_received = false;
bool current_state_received = false;

// Callback for /joint_desired_state topic
void desiredStateCallback(const smm_control::CustomJointState::ConstPtr& msg) {
    latest_desired_state = *msg;
    desired_state_received = true;
}

// Callback for /joint_current_state topic
void currentStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    latest_current_state = *msg;
    current_state_received = true;
}

// Function to compute the error and publish it
void computeAndPublishErrors(ros::Publisher& error_pub) {
    if (!desired_state_received || !current_state_received) {
        ROS_WARN_THROTTLE(1, "[updateErrorState_fasmc_simple/computeAndPublishErrors] Waiting for both desired and current states...");
        return;
    }

    // Ensure both messages have data for 3 joints
    if (latest_desired_state.position.size() != 3 || latest_current_state.position.size() != 3 ||
        latest_desired_state.velocity.size() != 3 || latest_current_state.velocity.size() != 3) {
        ROS_ERROR("Incorrect number of joints in the received messages. Expected 3.");
        return;
    }

    smm_control::FasmcError error_msg;
    
    // Compute position and velocity errors for each joint
    for (size_t i = 0; i < 3; ++i) {
        error_msg.position_error[i] = latest_desired_state.position[i] - latest_current_state.position[i];
        error_msg.velocity_error[i] = latest_desired_state.velocity[i] - latest_current_state.velocity[i];
    }

    // Publish the error message
    error_pub.publish(error_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateErrorState_fasmc_simple");
    ros::NodeHandle nh;

    // Subscriber for the desired and current joint states
    ros::Subscriber desired_state_sub = nh.subscribe("/joint_desired_state", 10, desiredStateCallback);
    ros::Subscriber current_state_sub = nh.subscribe("/joint_current_state", 10, currentStateCallback);

    // Publisher for the error state
    ros::Publisher error_pub = nh.advertise<smm_control::FasmcError>("/fasmc_error_state", 10);

    ros::Rate loop_rate(UPDATE_ERROR_STATE_RATE);

    while (ros::ok()) {
        computeAndPublishErrors(error_pub);  // Compute and publish position and velocity errors

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
