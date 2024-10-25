#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <smm_control/IdoscDesired.h>   
#include <smm_control/IdoscCurrent.h>   
#include <smm_control/IdoscError.h> 

geometry_msgs::Pose desired_pose;
geometry_msgs::Twist desired_twist;
geometry_msgs::Pose current_pose;
geometry_msgs::Twist current_twist;
bool desired_received = false;
bool current_received = false;

// Callback for the desired TCP state
void desiredStateCallback(const smm_control::IdoscDesired::ConstPtr& msg) {
    desired_pose = msg->pose_des;
    desired_twist = msg->twist_des;
    desired_received = true;
}

// Callback for the current TCP state
void currentStateCallback(const smm_control::IdoscCurrent::ConstPtr& msg) {
    current_pose = msg->pose_cur;
    current_twist = msg->twist_cur;
    current_received = true;
}

// Function to compute the error state (desired - current)
smm_control::IdoscError computeErrorState() {
    smm_control::IdoscError error_state;

    // Compute position error (desired - current)
    error_state.pose_error.position.x = desired_pose.position.x - current_pose.position.x;
    error_state.pose_error.position.y = desired_pose.position.y - current_pose.position.y;
    error_state.pose_error.position.z = desired_pose.position.z - current_pose.position.z;

    // Assuming no change in orientation for simplicity (modify as needed)
    error_state.pose_error.orientation.w = 1.0;
    error_state.pose_error.orientation.x = 0.0;
    error_state.pose_error.orientation.y = 0.0;
    error_state.pose_error.orientation.z = 0.0;

    // Compute velocity error (desired - current)
    error_state.twist_error.linear.x = desired_twist.linear.x - current_twist.linear.x;
    error_state.twist_error.linear.y = desired_twist.linear.y - current_twist.linear.y;
    error_state.twist_error.linear.z = desired_twist.linear.z - current_twist.linear.z;

    error_state.twist_error.angular.x = desired_twist.angular.x - current_twist.angular.x;
    error_state.twist_error.angular.y = desired_twist.angular.y - current_twist.angular.y;
    error_state.twist_error.angular.z = desired_twist.angular.z - current_twist.angular.z;

    return error_state;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateErrorState_idosc_simple");
    ros::NodeHandle nh;

    // Subscriber for the desired TCP state
    ros::Subscriber desired_state_sub = nh.subscribe("/tcp_desired_state", 10, desiredStateCallback);

    // Subscriber for the current TCP state
    ros::Subscriber current_state_sub = nh.subscribe("/tcp_current_state", 10, currentStateCallback);

    // Publisher for the error state
    ros::Publisher error_state_pub = nh.advertise<smm_control::IdoscError>("/idosc_error_state", 10);

    ros::Rate loop_rate(1000);  // 10 Hz

    while (ros::ok()) {
        if (desired_received && current_received) {
            // Compute the error state
            smm_control::IdoscError error_state = computeErrorState();

            // Publish the error state
            error_state_pub.publish(error_state);

            // Log the error state
            ROS_INFO("[updateErrorState_idosc_simple] Publishing error state: position error (x: %f, y: %f, z: %f), velocity error (x: %f, y: %f, z: %f)",
                     error_state.pose_error.position.x,
                     error_state.pose_error.position.y,
                     error_state.pose_error.position.z,
                     error_state.twist_error.linear.x,
                     error_state.twist_error.linear.y,
                     error_state.twist_error.linear.z);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}