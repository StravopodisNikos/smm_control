#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <smm_control/RobotTorques.h>  
#include "smm_control/timing.h"

const float MAX_TORQUE_LIMIT = 100.0;

float limitTorque(float torque) {
    if (torque > MAX_TORQUE_LIMIT) {
        return MAX_TORQUE_LIMIT;
    } else if (torque < -MAX_TORQUE_LIMIT) {
        return -MAX_TORQUE_LIMIT;
    }
    return torque;
}

// Callback to handle incoming torque data
void torquesCallback(const smm_control::RobotTorques::ConstPtr& msg, ros::Publisher pub1, ros::Publisher pub2, ros::Publisher pub3) {
    // Publish torques to the individual joint effort controllers

    std_msgs::Float64 torque_msg;

    // Joint 1 effort
    torque_msg.data = limitTorque(msg->torques[0]);
    pub1.publish(torque_msg);
    // Joint 2 effort
    torque_msg.data = limitTorque(msg->torques[1]);
    pub2.publish(torque_msg);
    // Joint 3 effort
    torque_msg.data = limitTorque(msg->torques[2]);
    pub3.publish(torque_msg);

    ROS_INFO("[robot_effort_publisher/torquesCallback] Published torques - Joint 1: %f, Joint 2: %f, Joint 3: %f", 
         msg->torques[0], 
         msg->torques[1], 
         msg->torques[2]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_torque_command_publisher");
    ros::NodeHandle nh;

    // Publishers for the individual joint effort controllers
    ros::Publisher joint1_effort_pub = nh.advertise<std_msgs::Float64>("/smm_ros_gazebo/joint1_effort_controller/command", 10);
    ros::Publisher joint2_effort_pub = nh.advertise<std_msgs::Float64>("/smm_ros_gazebo/joint2_effort_controller/command", 10);
    ros::Publisher joint3_effort_pub = nh.advertise<std_msgs::Float64>("/smm_ros_gazebo/joint3_effort_controller/command", 10);

    // Subscriber to the /idosc_torques topic
    ros::Subscriber torques_sub = nh.subscribe<smm_control::RobotTorques>("/robot_torque_command", 10, 
        boost::bind(torquesCallback, _1, joint1_effort_pub, joint2_effort_pub, joint3_effort_pub));

    // Run in loop
    ros::Rate loop_rate(UPDATE_TORQUE_COMMAND_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}