#include <ros/ros.h>
#include <smm_control/FasmcAdaptiveParams.h>
#include <smm_control/FasmcFuzzyParams.h>
#include <smm_control/FasmcTorques.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

// Global variables for parameters
boost::array<double, 3> eta = {0.0, 0.0, 0.0};
boost::array<double, 3> epsilon = {0.0, 0.0, 0.0};
boost::array<double, 3> gamma_h = {0.0, 0.0, 0.0};
geometry_msgs::Vector3 sliding_surface;

ros::Publisher torque_pub;

// Callback for /fasmc_adaptive_params
void adaptiveParamsCallback(const smm_control::FasmcAdaptiveParams::ConstPtr& msg) {
    eta = msg->eta;
    epsilon = msg->epsilon;
}

// Callback for /fasmc_sliding_surface
void slidingSurfaceCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    sliding_surface = *msg;
}

// Callback for /fasmc_fuzzy_params
void fuzzyParamsCallback(const smm_control::FasmcFuzzyParams::ConstPtr& msg) {
    gamma_h = msg->gamma;
}

// Function to calculate torque_smc and publish it
void calculateTorque() {
    smm_control::FasmcTorques torque_msg;

    // Calculate torque for each joint
    for (int i = 0; i < 3; ++i) {
        double s = (i == 0 ? sliding_surface.x : (i == 1 ? sliding_surface.y : sliding_surface.z));
        torque_msg.torques[i] = -eta[i] * std::tanh(gamma_h[i] * s) - epsilon[i] * s;
    }

    // Publish the calculated torques
    torque_pub.publish(torque_msg);

    ROS_INFO_STREAM("[updateRobustTorque_fasmc_simple/calculateTorque] Calculated Torques: [" 
                << torque_msg.torques[0] << ", " 
                << torque_msg.torques[1] << ", " 
                << torque_msg.torques[2] << "]");

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateRobustTorque_fasmc_simple");
    ros::NodeHandle nh;

    // Subscribe to the relevant topics
    ros::Subscriber adaptive_params_sub = nh.subscribe("/fasmc_adaptive_params", 10, adaptiveParamsCallback);
    ros::Subscriber sliding_surface_sub = nh.subscribe("/fasmc_sliding_surface", 10, slidingSurfaceCallback);
    ros::Subscriber fuzzy_params_sub = nh.subscribe("/fasmc_fuzzy_params", 10, fuzzyParamsCallback);

    // Publisher for calculated torque values
    torque_pub = nh.advertise<smm_control::FasmcTorques>("/fasmc_robust_term", 10);

    ros::Rate rate(10);  // Adjust rate as needed
    while (ros::ok()) {
        ros::spinOnce();
        calculateTorque();
        rate.sleep();
    }

    return 0;
}
