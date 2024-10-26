#include <ros/ros.h>
#include <smm_control/FasmcFuzzyParams.h>
#include <smm_control/FasmcError.h>
#include <smm_control/FasmcAdaptiveParams.h>  // Custom message with eta and epsilon
#include <cmath>

std::vector<double> eta_0(3, 0.01);
std::vector<double> epsilon_0(3, 0.01);
boost::array<double, 3> alpha = {0.0, 0.0, 0.0};
boost::array<double, 3> beta = {0.0, 0.0, 0.0};
boost::array<double, 3> position_error = {0.0, 0.0, 0.0};

ros::Publisher adaptive_params_pub;

// Callback for /fasmc_fuzzy_params
void fuzzyParamsCallback(const smm_control::FasmcFuzzyParams::ConstPtr& msg) {
    alpha = msg->alpha;
    beta = msg->beta;
}

// Callback for /fasmc_error_state
void errorStateCallback(const smm_control::FasmcError::ConstPtr& msg) {
    position_error = msg->position_error;
}

// Function to calculate eta and epsilon values
void calculateEtaEpsilon() {
    smm_control::FasmcAdaptiveParams eta_epsilon_msg;

    for (int i = 0; i < 3; ++i) {
        double abs_position_error = std::abs(position_error[i]);
        eta_epsilon_msg.eta[i] = eta_0[i] * (1.0 + alpha[i] * abs_position_error);
        eta_epsilon_msg.epsilon[i] = epsilon_0[i] * (1.0 + beta[i] * abs_position_error);
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
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateParams_fasmc_simple");
    ros::NodeHandle nh;

    // Load fixed parameters from YAML file
    if (!loadParameters(nh)) {
        ROS_ERROR("[updateParams_fasmc_simple] Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscribe to topics
    ros::Subscriber fuzzy_params_sub = nh.subscribe("/fasmc_fuzzy_params", 10, fuzzyParamsCallback);
    ros::Subscriber error_state_sub = nh.subscribe("/fasmc_error_state", 10, errorStateCallback);

    // Publisher for calculated eta and epsilon values
    adaptive_params_pub = nh.advertise<smm_control::FasmcAdaptiveParams>("/fasmc_adaptive_params", 10);

    ros::Rate rate(10);  // Adjust rate as needed
    while (ros::ok()) {
        ros::spinOnce();
        calculateEtaEpsilon();
        rate.sleep();
    }

    return 0;
}
