#include <ros/ros.h>
#include <smm_control/GetOperationalSpaceMatrices.h>
#include <Eigen/Dense>
#include <iostream>
#include "smm_control/timing.h"
#include <std_msgs/Float64MultiArray.h>

// Global variables
Eigen::Matrix3f current_Lambda = Eigen::Matrix3f::Zero();
Eigen::Matrix3f previous_Lambda = Eigen::Matrix3f::Zero();
ros::Time previous_time;
ros::Time current_time;

// Function to get the Lambda matrix from the service
bool getLambdaMatrix(ros::NodeHandle& nh) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetOperationalSpaceMatrices>("GetOperationalSpaceMatrices");
    smm_control::GetOperationalSpaceMatrices srv;

    // Set the request flag for Lambda matrix
    srv.request.get_Lambda = true;

    if (client.call(srv)) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                current_Lambda(i, j) = srv.response.Lambda[i * 3 + j];
            }
        }
        return true;
    } else {
        ROS_ERROR("[updateDtLambda/getLambdaMatrix] Failed to call GetOperationalSpaceMatrices service.");
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateDtLambda");
    ros::NodeHandle nh;

    // Publisher for Lambda time derivative
    ros::Publisher lambda_dot_pub = nh.advertise<std_msgs::Float64MultiArray>("/lambda_time_derivative", 10);

    // Timing and rate
    ros::Rate loop_rate(DT_LAMBDA_UPDATE_RATE);
    previous_time = ros::Time::now();

    while (ros::ok()) {
        current_time = ros::Time::now();

        // Get the Lambda matrix from the service
        if (!getLambdaMatrix(nh)) {
            ROS_WARN("[updateDtLambda] Unable to update Lambda matrix. Retrying...");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // Calculate time difference
        double dt = (current_time - previous_time).toSec();
        if (dt <= 0.0) {
            ROS_WARN("[updateDtLambda] Non-positive time difference detected. Skipping iteration.");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // Calculate the time derivative of the Lambda matrix
        Eigen::Matrix3f lambda_dot = (current_Lambda - previous_Lambda) / dt;

        // Prepare the message for publishing
        std_msgs::Float64MultiArray lambda_dot_msg;
        lambda_dot_msg.data.resize(9);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                lambda_dot_msg.data[i * 3 + j] = lambda_dot(i, j);
            }
        }

        // Publish the message
        lambda_dot_pub.publish(lambda_dot_msg);

        // Debug output
        ROS_INFO_STREAM("Lambda Time Derivative:\n" << lambda_dot);

        // Update previous values
        previous_Lambda = current_Lambda;
        previous_time = current_time;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
