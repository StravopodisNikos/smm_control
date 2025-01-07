#include <ros/ros.h>
#include <smm_control/GetOperationalSpaceMatrices.h>
#include <Eigen/Dense>
#include <iostream>
#include <std_msgs/Float32.h>
#include "smm_control/timing.h"

// Global variables
Eigen::Matrix3f LambdaMatrix = Eigen::Matrix3f::Zero();
Eigen::Matrix3f InverseLambdaMatrix = Eigen::Matrix3f::Zero();
float LambdaCondNumber;
float LambdaInverseNorm;
Eigen::Matrix3f singular_values;
Eigen::JacobiSVD<Eigen::Matrix3f> svd(Eigen::Matrix3f::Zero(), Eigen::ComputeFullU | Eigen::ComputeFullV);

constexpr float LAMBDA_COND_THRES = 5.0f;
constexpr float DLS_LAMBDA = 0.005f;  // Damping factor for DLS inversion

// Function to get the Lambda matrix from the service
bool getLambdaMatrix(ros::NodeHandle& nh) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetOperationalSpaceMatrices>("GetOperationalSpaceMatrices");
    smm_control::GetOperationalSpaceMatrices srv;

    // Set the request flag for Lambda matrix
    srv.request.get_Lambda = true;

    if (client.call(srv)) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                LambdaMatrix(i, j) = srv.response.Lambda[i * 3 + j];
            }
        }
        return true;
    } else {
        ROS_ERROR("[updateInverseLambda/getLambdaMatrix] Failed to call GetOperationalSpaceMatrices service.");
        return false;
    }
}

// Function to calculate the condition number of a matrix
float calculateConditionNumber(const Eigen::Matrix3f& matrix) {
    svd.compute(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

    singular_values = svd.singularValues().asDiagonal();

    float sigma_max = svd.singularValues()(0);
    float sigma_min = svd.singularValues()(2);

    LambdaCondNumber = sigma_max / sigma_min;
    
    return LambdaCondNumber;
}

// Function to calculate the Damped Least Squares (DLS) inverse
Eigen::Matrix3f calculateAdaptiveDLSInverse(const Eigen::Matrix3f& matrix) {

    Eigen::Matrix3f adaptive_inverse = Eigen::Matrix3f::Zero();

    // Adjust the damping factor based on the condition number
    float adaptive_damping_factor = DLS_LAMBDA;
    if (LambdaCondNumber > LAMBDA_COND_THRES) {
        adaptive_damping_factor *= (LambdaCondNumber / LAMBDA_COND_THRES);
    }

    // Compute the adaptive DLS pseudo-inverse
    for (int i = 0; i < 3; ++i) {
        float sigma = singular_values(i, i);
        singular_values(i, i) = sigma / (sigma * sigma + adaptive_damping_factor * adaptive_damping_factor);
    }

    adaptive_inverse = svd.matrixV() * singular_values * svd.matrixU().transpose();
    return adaptive_inverse;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "updateInverseLambda");
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher lambda_cond_pub = nh.advertise<std_msgs::Float32>("/lambda_condition_number", 10);
    ros::Publisher inv_lambda_norm_pub = nh.advertise<std_msgs::Float32>("/inverse_lambda_norm", 10);

    // Timing and rate
    ros::Rate loop_rate(INVERSE_LAMBDA_UPDATE_RATE);

    while (ros::ok()) {
        // Get the Lambda matrix from the service
        if (!getLambdaMatrix(nh)) {
            ROS_WARN("[updateInverseLambda] Unable to update Lambda matrix. Retrying...");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // Calculate the condition number
        LambdaCondNumber = calculateConditionNumber(LambdaMatrix);

        // Publish the condition number
        std_msgs::Float32 cond_msg;
        cond_msg.data = LambdaCondNumber;
        lambda_cond_pub.publish(cond_msg);

        // Check the condition number and calculate the inverse
        if (LambdaCondNumber < LAMBDA_COND_THRES) {
            // Simple inverse
            InverseLambdaMatrix = LambdaMatrix.inverse();
        } else {
            // DLS inverse
            ROS_WARN("[updateInverseLambda] Lambda matrix is ill-conditioned. Using DLS inverse.");
            InverseLambdaMatrix = calculateAdaptiveDLSInverse(LambdaMatrix);
        }

        // Calculate the norm of the inverse Lambda matrix
        LambdaInverseNorm = InverseLambdaMatrix.norm();

        // Publish the norm of the inverse
        std_msgs::Float32 norm_msg;
        norm_msg.data = LambdaInverseNorm;
        inv_lambda_norm_pub.publish(norm_msg);

        // Loop timing
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
