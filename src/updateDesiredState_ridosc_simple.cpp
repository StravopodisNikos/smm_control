#include <ros/ros.h>
#include <smm_control/CustomTcpState.h>
#include <smm_control/IdoscDesired.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "smm_control/timing.h"

class DesiredStateUpdater {
public:
    DesiredStateUpdater(ros::NodeHandle& nh) {
        // Subscriber to /tcp_desired_state
        tcp_state_sub_ = nh.subscribe("/tcp_desired_state", 10, &DesiredStateUpdater::tcpStateCallback, this);

        // Publisher to /ridosc_desired_state
        ridosc_state_pub_ = nh.advertise<smm_control::IdoscDesired>("/ridosc_desired_state", 10);
    }

    void tcpStateCallback(const smm_control::CustomTcpState::ConstPtr& msg) {
        // Populate the desired state message
        smm_control::IdoscDesired desired_state_msg;

        // Map position to Pose
        desired_state_msg.pose_des.position.x = msg->position[0];
        desired_state_msg.pose_des.position.y = msg->position[1];
        desired_state_msg.pose_des.position.z = msg->position[2];

        // Set orientation to identity quaternion (no rotation)
        desired_state_msg.pose_des.orientation.x = 0.0;
        desired_state_msg.pose_des.orientation.y = 0.0;
        desired_state_msg.pose_des.orientation.z = 0.0;
        desired_state_msg.pose_des.orientation.w = 1.0;

        // Map velocity to Twist
        desired_state_msg.twist_des.linear.x = msg->velocity[0];
        desired_state_msg.twist_des.linear.y = msg->velocity[1];
        desired_state_msg.twist_des.linear.z = msg->velocity[2];

        // Set angular velocity to zero
        desired_state_msg.twist_des.angular.x = 0.0;
        desired_state_msg.twist_des.angular.y = 0.0;
        desired_state_msg.twist_des.angular.z = 0.0;

        // Publish the message
        ridosc_state_pub_.publish(desired_state_msg);
    }

private:
    ros::Subscriber tcp_state_sub_;
    ros::Publisher ridosc_state_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "updateDesiredState_ridosc_simple");
    ros::NodeHandle nh;

    // Create an instance of the node
    DesiredStateUpdater updater(nh);

    // Set the loop rate
    ros::Rate loop_rate(UPDATE_DESIRED_STATE_RATE);

    // Main loop
    while (ros::ok()) {
        ros::spinOnce();  // Process callbacks
        loop_rate.sleep();  // Sleep to maintain the loop rate
    }

    return 0;
}
