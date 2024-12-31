#include <ros/ros.h>
#include <smm_control/IdoscError.h>
#include <smm_control/FasmcAdaptiveParams.h>  // Custom message with eta and epsilon, same for fasmsc and idosc/ridosc! 
#include <cmath>
#include "smm_control/timing.h"

// Initialization only - Params change from yaml files loaded
std::vector<double> eta_0(3, 0.01);   
std::vector<double> epsilon_0(3, 0.01); 
std::vector<double> alpha(3, 0.01);       
std::vector<double> beta(3, 0.01);        
boost::array<double, 3> position_error = {0.0, 0.0, 0.0};

ros::Publisher adaptive_params_pub;

// Callback for /fasmc_error_state
void errorStateCallback(const smm_control::IdoscError::ConstPtr& msg) {
    // Extract position error from the pose_error field
    position_error[0] = msg->pose_error.position.x;
    position_error[1] = msg->pose_error.position.y;
    position_error[2] = msg->pose_error.position.z;

    // Log the extracted position error (optional for debugging)
    ROS_INFO("[updateParams_ridosc_simple/errorStateCallback] Position Error: x=%.3f, y=%.3f, z=%.3f", 
             position_error[0], position_error[1], position_error[2]);
}

// Function to calculate eta and epsilon values
void calculateEtaEpsilon() {
    smm_control::FasmcAdaptiveParams eta_epsilon_msg;

    for (int i = 0; i < 3; ++i) {
        double abs_position_error = std::abs(position_error[i]);

        if (abs_position_error < 0.1) {
            eta_epsilon_msg.eta[i] = eta_0[i] - alpha[i] * abs_position_error;
            eta_epsilon_msg.epsilon[i] = epsilon_0[i] - beta[i] * abs_position_error;
            //eta_epsilon_msg.eta[i] = eta_0[i];
            //eta_epsilon_msg.epsilon[i] = epsilon_0[i];
        } else {
            eta_epsilon_msg.eta[i] = eta_0[i] + alpha[i] * abs_position_error;
            eta_epsilon_msg.epsilon[i] = epsilon_0[i] + beta[i] * abs_position_error;
            //eta_epsilon_msg.eta[i] = eta_0[i];
            //eta_epsilon_msg.epsilon[i] = epsilon_0[i];
        }
    }

    // Publish the calculated eta and epsilon values
    adaptive_params_pub.publish(eta_epsilon_msg);
}

bool loadParameters(ros::NodeHandle& nh) {
    if (nh.getParam("/eta_0", eta_0)) {
        if (eta_0.size() != 3) {
            ROS_ERROR("eta_0 should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded eta_0 from parameter server.");
    } else {
        ROS_ERROR("Failed to load eta_0 from parameter server.");
        return false;
    }
    if (nh.getParam("/epsilon_0", epsilon_0)) {
        if (epsilon_0.size() != 3) {
            ROS_ERROR("epsilon_0 should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded epsilon_0 from parameter server.");
    } else {
        ROS_ERROR("Failed to load epsilon_0 from parameter server.");
        return false;
    }
    if (nh.getParam("/alpha", alpha)) {
        if (alpha.size() != 3) {
            ROS_ERROR("alpha should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded alpha from parameter server.");
    } else {
        ROS_ERROR("Failed to load lambda_0 from parameter server.");
        return false;
    }
    if (nh.getParam("/beta", beta)) {
        if (beta.size() != 3) {
            ROS_ERROR("beta should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded beta from parameter server.");
    } else {
        ROS_ERROR("Failed to load beta from parameter server.");
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateParams_ridosc_simple");
    ros::NodeHandle nh;

    // Load fixed parameters from YAML file
    if (!loadParameters(nh)) {
        ROS_ERROR("[updateParams_ridosc_simple] Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscribe to topics
    ros::Subscriber error_state_sub = nh.subscribe("/ridosc_error_state", 10, errorStateCallback);

    // Publisher for calculated eta and epsilon values
    adaptive_params_pub = nh.advertise<smm_control::FasmcAdaptiveParams>("/ridosc_adaptive_params", 10); // same params as fasmc

    ros::Rate rate(UPDATE_PARAMS_RATE);  // Adjust rate as needed
    while (ros::ok()) {
        ros::spinOnce();
        calculateEtaEpsilon();
        rate.sleep();
    }

    return 0;
}
