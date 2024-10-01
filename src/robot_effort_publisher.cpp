#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <smm_control/IdoscTorques.h>  

// Callback to handle incoming torque data
void torquesCallback(const smm_control::IdoscTorques::ConstPtr& msg, ros::Publisher pub1, ros::Publisher pub2, ros::Publisher pub3) {
    // Publish torques to the individual joint effort controllers

    std_msgs::Float64 torque_msg;

    // Joint 1 effort
    torque_msg.data = msg->robot_torques[0];
    pub1.publish(torque_msg);
    ROS_INFO("[robot_effort_publisher] Published torque for joint 1: %f", torque_msg.data);

    // Joint 2 effort
    torque_msg.data = msg->robot_torques[1];
    pub2.publish(torque_msg);
    ROS_INFO("[robot_effort_publisher] Published torque for joint 2: %f", torque_msg.data);

    // Joint 3 effort
    torque_msg.data = msg->robot_torques[2];
    pub3.publish(torque_msg);
    ROS_INFO("[robot_effort_publisher] Published torque for joint 3: %f", torque_msg.data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_effort_publisher");
    ros::NodeHandle nh;

    // Publishers for the individual joint effort controllers
    ros::Publisher joint1_effort_pub = nh.advertise<std_msgs::Float64>("/smm_ros_gazebo/joint1_effort_controller/command", 10);
    ros::Publisher joint2_effort_pub = nh.advertise<std_msgs::Float64>("/smm_ros_gazebo/joint2_effort_controller/command", 10);
    ros::Publisher joint3_effort_pub = nh.advertise<std_msgs::Float64>("/smm_ros_gazebo/joint3_effort_controller/command", 10);

    // Subscriber to the /idosc_torques topic
    ros::Subscriber torques_sub = nh.subscribe<smm_control::IdoscTorques>("/idosc_torques", 10, 
        boost::bind(torquesCallback, _1, joint1_effort_pub, joint2_effort_pub, joint3_effort_pub));

    ros::spin();  // Spin to keep the node alive and responsive to callbacks

    return 0;
}