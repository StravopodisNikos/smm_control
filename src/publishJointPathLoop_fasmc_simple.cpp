#include <ros/ros.h>
#include <smm_control/CustomJointState.h>  // Ensure this path matches your package structure
#include <vector>
#include <string>
#include "smm_control/timing.h"

// Global vector to hold each joint state message
std::vector<smm_control::CustomJointState> joint_path;
ros::Publisher desired_state_pub;
int current_sequence_index = 0;  // Index for tracking the current waypoint

// Function to load the parameters from the YAML file on the parameter server
bool loadParameters(ros::NodeHandle& nh) {
    std::vector<std::string> position_names;
    std::vector<std::string> velocity_names;
    std::vector<std::string> acceleration_names;

    // Retrieve names for positions, velocities, and accelerations
    if (!nh.getParam("/positions/names", position_names) || 
        !nh.getParam("/velocities/names", velocity_names) || 
        !nh.getParam("/accelerations/names", acceleration_names)) {
        ROS_ERROR("Failed to load names from parameter server.");
        return false;
    }

    // Iterate over each position, velocity, and acceleration entry
    for (size_t i = 0; i < position_names.size(); ++i) {
        // Load position data
        std::vector<double> position_data;
        if (!nh.getParam("/positions/" + position_names[i], position_data) || position_data.size() != 3) {
            ROS_ERROR("Failed to load position data for %s", position_names[i].c_str());
            return false;
        }

        // Load velocity data
        std::vector<double> velocity_data;
        if (!nh.getParam("/velocities/" + velocity_names[i], velocity_data) || velocity_data.size() != 3) {
            ROS_ERROR("Failed to load velocity data for %s", velocity_names[i].c_str());
            return false;
        }

        // Load acceleration data
        std::vector<double> acceleration_data;
        if (!nh.getParam("/accelerations/" + acceleration_names[i], acceleration_data) || acceleration_data.size() != 3) {
            ROS_ERROR("Failed to load acceleration data for %s", acceleration_names[i].c_str());
            return false;
        }

        // Create a CustomJointState message and populate fields
        smm_control::CustomJointState joint_state;
        joint_state.name = {"joint1", "joint2", "joint3"};
        joint_state.position = {position_data[0], position_data[1], position_data[2]};
        joint_state.velocity = {velocity_data[0], velocity_data[1], velocity_data[2]};
        joint_state.acceleration = {acceleration_data[0], acceleration_data[1], acceleration_data[2]};

        // Add to joint path
        joint_path.push_back(joint_state);
    }

    return true;
}

// Function to publish the current desired state
void publishDesiredState() {
    if (joint_path.empty()) {
        ROS_WARN("No joint states available to publish.");
        return;
    }

    // Get the current waypoint based on the sequence index
    const smm_control::CustomJointState& desired_msg = joint_path[current_sequence_index];

    // Publish the desired state message
    desired_state_pub.publish(desired_msg);

    // Move to the next waypoint in the sequence
    current_sequence_index = (current_sequence_index + 1) % joint_path.size();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "publishJointPathLoop_fasmc_simple");
    ros::NodeHandle nh;

    // Load parameters
    if (!loadParameters(nh)) {
        ROS_ERROR("Failed to load joint trajectory data.");
        return -1;
    }

    // Initialize the publisher for /joint_desired_state
    desired_state_pub = nh.advertise<smm_control::CustomJointState>("/joint_desired_state", 10);

    publishDesiredState();

    ros::Rate loop_rate(UPDATE_JOINT_PATH_LOOP_RATE);
    
    while (ros::ok()) {
        publishDesiredState();  // Publish the current desired state
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
