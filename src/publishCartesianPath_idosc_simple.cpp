#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <smm_control/IdoscDesired.h>  
#include <vector>
#include <string>

// Define global variables to hold the loaded data
std::vector<geometry_msgs::Pose> waypoints;
std::vector<geometry_msgs::Twist> velocities;
int current_index = 0;
ros::Publisher desired_state_pub;

// Function to load the waypoints, velocities, and configurations from the parameter server
bool loadParameters(ros::NodeHandle& nh) {
    // Load waypoints
    std::vector<std::string> waypoint_names;
    if (!nh.getParam("/waypoints/names", waypoint_names)) {
        ROS_ERROR("[publishCartesianPath_idosc_simple/loadParameters] Failed to load /waypoints/names from parameter server.");
        return false;
    }

    for (const auto& waypoint_name : waypoint_names) {
        std::vector<double> waypoint_data;
        if (nh.getParam("/waypoints/" + waypoint_name, waypoint_data)) {
            if (waypoint_data.size() == 3) {  // Ensure waypoint contains 3 values
                geometry_msgs::Pose pose;
                pose.position.x = waypoint_data[0];
                pose.position.y = waypoint_data[1];
                pose.position.z = waypoint_data[2];
                pose.orientation.w = 1.0;  // Assuming identity orientation
                waypoints.push_back(pose);
            } else {
                ROS_ERROR("[publishCartesianPath_idosc_simple/loadParameters] Waypoint %s does not contain 3 values.", waypoint_name.c_str());
                return false;
            }
        } else {
            ROS_ERROR("[publishCartesianPath_idosc_simple/loadParameters] Failed to load waypoint %s from parameter server.", waypoint_name.c_str());
            return false;
        }
    }

    // Load velocities
    std::vector<std::string> velocity_names;
    if (!nh.getParam("/velocities/names", velocity_names)) {
        ROS_ERROR("Failed to load /velocities/names from parameter server.");
        return false;
    }

    for (const auto& velocity_name : velocity_names) {
        std::vector<double> velocity_data;
        if (nh.getParam("/velocities/" + velocity_name, velocity_data)) {
            if (velocity_data.size() == 3) {  // Ensure velocity contains 3 values
                geometry_msgs::Twist twist;
                twist.linear.x = velocity_data[0];
                twist.linear.y = velocity_data[1];
                twist.linear.z = velocity_data[2];
                velocities.push_back(twist);
            } else {
                ROS_ERROR("[publishCartesianPath_idosc_simple/loadParameters] Velocity %s does not contain 3 values.", velocity_name.c_str());
                return false;
            }
        } else {
            ROS_ERROR("[publishCartesianPath_idosc_simple/loadParameters] Failed to load velocity %s from parameter server.", velocity_name.c_str());
            return false;
        }
    }

    return true;
}

// Function to publish the current desired state
void publishDesiredState() {
    smm_control::IdoscDesired desired_msg;

    if (current_index >= waypoints.size()) {
        current_index = waypoints.size() - 1;  // Keep sending the last waypoint
    }

    // Set the desired pose and velocity
    desired_msg.pose_des = waypoints[current_index];
    desired_msg.twist_des = velocities[current_index];

    // Publish the desired state message
    desired_state_pub.publish(desired_msg);

    ROS_INFO("[publishCartesianPath_idosc_simple/publishDesiredState] Publishing desired state: position (x: %f, y: %f, z: %f), velocity (x: %f, y: %f, z: %f)",
             desired_msg.pose_des.position.x,
             desired_msg.pose_des.position.y,
             desired_msg.pose_des.position.z,
             desired_msg.twist_des.linear.x,
             desired_msg.twist_des.linear.y,
             desired_msg.twist_des.linear.z);

    current_index++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "publishCartesianPath_idosc_simple");
    ros::NodeHandle nh;

    // Load parameters
    if (!loadParameters(nh)) {
        ROS_ERROR("[publishCartesianPath_idosc_simple] Failed to load parameters.");
        return -1;
    }

    // Publisher for the desired TCP position and velocity
    desired_state_pub = nh.advertise<smm_control::IdoscDesired>("/tcp_desired_state", 10);

    ros::Rate loop_rate(0.5);  // 0.5 Hz -> 2 seconds

    while (ros::ok()) {
        publishDesiredState();  // Publish the next desired state
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}