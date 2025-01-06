#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <smm_control/GetJacobians.h>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include "smm_control/timing.h"

Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
float kappa_neglect;
float kappa_ill_lim;
float mbs_bounded;
float w_j;
float w_m;

// Function to compute the uncertainty metric
float computeUncertainty() {
    // Check bounds of mbs
    if (mbs_bounded < 0.0f || mbs_bounded > 1.0f) {
        throw std::invalid_argument("[updateUncertaintyFactor/computeUncertainty] Index mbs should be in [0, 1]");
    }

    // Compute singular values of the Jacobian
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    float sigma_max = svd.singularValues()(0);
    float sigma_min = svd.singularValues()(2);

    // Condition number
    float kappa = sigma_max / sigma_min;

    // Normalize the condition number to [0, 1]
    float normalized_kappa = std::max(0.0f, std::min(1.0f, (kappa - kappa_neglect) / (kappa_ill_lim - kappa_neglect)));

    // Compute the uncertainty
    float uncertainty = w_j * normalized_kappa + w_m * (1.0f - mbs_bounded);

    return uncertainty;
}

// Function to get the operational Jacobian from the service
bool getOperationalJacobian(ros::NodeHandle& nh, Eigen::Matrix3f& J) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetJacobians>("GetOperationalJacobians");
    smm_control::GetJacobians srv;

    // Request only the operational Jacobian
    srv.request.get_op_jacobian = true;
    srv.request.get_inv_op_jacobian = false;
    srv.request.get_dt_op_jacobian = false;

    if (client.call(srv)) {
        // Convert the flat array back to 3x3 matrix
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                J(i, j) = srv.response.op_jacobian[i * 3 + j];
            }
        }
        ROS_INFO_STREAM("Operational Jacobian successfully received:\n" << J);
        return true;
    } else {
        ROS_ERROR("Failed to call service GetOperationalJacobians.");
        return false;
    }
}

// Function to load parameters from the YAML file
bool loadParameters(ros::NodeHandle& nh, float& mbs, float& kappa_min, float& kappa_max, float& w_jacob, float& w_mbs) {
    try {
        // Load mbs parameter
        if (!nh.getParam("mbs", mbs)) {
            ROS_WARN("[loadParameters] Parameter 'mbs' not found. Using default value 0.3525.");
            mbs = 0.3525f;
        }

        // Load kappa_min parameter
        if (!nh.getParam("kappa_min", kappa_min)) {
            ROS_WARN("[loadParameters] Parameter 'kappa_min' not found. Using default value 1.0.");
            kappa_min = 1.0f;
        }

        // Load kappa_max parameter
        if (!nh.getParam("kappa_max", kappa_max)) {
            ROS_WARN("[loadParameters] Parameter 'kappa_max' not found. Using default value 10.0.");
            kappa_max = 10.0f;
        }

        // Load w_jacob parameter
        if (!nh.getParam("w_jacob", w_jacob)) {
            ROS_WARN("[loadParameters] Parameter 'w_jacob' not found. Using default value 0.5.");
            w_jacob = 0.5f;
        }

        // Load w_mbs parameter
        if (!nh.getParam("w_mbs", w_mbs)) {
            ROS_WARN("[loadParameters] Parameter 'w_mbs' not found. Using default value 0.5.");
            w_mbs = 0.5f;
        }

        // Validation checks
        if (mbs < 0.0f || mbs > 1.0f) {
            ROS_ERROR("[loadParameters] Invalid value for 'mbs'. It must be in [0, 1].");
            return false;
        }

        if (kappa_min < 0.0f || kappa_max <= kappa_min) {
            ROS_ERROR("[loadParameters] Invalid values for 'kappa_min' or 'kappa_max'. Ensure kappa_max > kappa_min and both are non-negative.");
            return false;
        }

        if (w_jacob < 0.0f || w_jacob > 1.0f || w_mbs < 0.0f || w_mbs > 1.0f || (w_jacob + w_mbs != 1.0f)) {
            ROS_ERROR("[loadParameters] Invalid weight distribution for 'w_jacob' and 'w_mbs'. Ensure 0 <= w_jacob, w_mbs <= 1 and their sum is 1.");
            return false;
        }

        return true; // Successfully loaded all parameters
    } catch (const std::exception& e) {
        ROS_ERROR("[loadParameters] Exception caught while loading parameters: %s", e.what());
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateUncertaintyFactor");
    ros::NodeHandle nh;

    if (!loadParameters(nh, mbs_bounded, kappa_neglect, kappa_ill_lim, w_j, w_m)) {
        ROS_ERROR("[updateUncertaintyFactor] Failed to load parameters. Shutting down node.");
        return -1;
    }

    // Publisher for the uncertainty factor
    ros::Publisher uncertainty_pub = nh.advertise<std_msgs::Float32>("/uncertainty_factor", 10);

    // Loop rate
    ros::Rate loop_rate(UPDATE_UNCERTAINTY_RATE); 
    while (ros::ok()) {
        std_msgs::Float32 uncertainty_msg;

        try {
            // Fetch the Jacobian from the service
            if (getOperationalJacobian(nh, J)) {
                // Compute the uncertainty factor
                uncertainty_msg.data = computeUncertainty();

                // Publish the uncertainty factor
                uncertainty_pub.publish(uncertainty_msg);

                ROS_INFO_STREAM("Published uncertainty factor: " << uncertainty_msg.data);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error computing uncertainty: " << e.what());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
