#include <ros/ros.h>
#include <smm_control/RidoscAdaptiveParams.h>
#include <smm_control/FasmcTorques.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include "smm_control/timing.h"

// Global variables for parameters
std::vector<double> gamma_h(3, 0.01); 
geometry_msgs::Vector3 sliding_surface;
// Local variables to hold the adaptive parameters
std::array<double, 3> qi_eta = {0.0, 0.0, 0.0};
std::array<double, 3> pi_epsilon = {0.0, 0.0, 0.0};

ros::Publisher torque_pub;

void adaptiveParamsCallback(const smm_control::RidoscAdaptiveParams::ConstPtr& msg) {
    for (size_t i = 0; i < 3; ++i) {
        qi_eta[i] = msg->qi[i];
        pi_epsilon[i] = msg->pi[i];
    }
}

// Callback for /fasmc_sliding_surface
void slidingSurfaceCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    sliding_surface = *msg;
}

// Function to calculate torque_smc and publish it
void calculateTorque() {
    smm_control::FasmcTorques torque_msg;

    // Calculate torque for each joint
    for (int i = 0; i < 3; ++i) {
        double s = (i == 0 ? sliding_surface.x : (i == 1 ? sliding_surface.y : sliding_surface.z));
        torque_msg.torques[i] = qi_eta[i] * std::tanh( gamma_h[i] * s) + pi_epsilon[i] * s; // based on book p.305
        //torque_msg.torques[i] =  qi_eta[i] * std::copysign(1.0, s) + pi_epsilon[i] * s;      // sign function - more aggresive
    }

    // Publish the calculated torques
    torque_pub.publish(torque_msg);
    /*
    ROS_INFO_STREAM("[updateRobustTorque_uridosc_simple/calculateTorque] Calculated Torques: [" 
                << torque_msg.torques[0] << ", " 
                << torque_msg.torques[1] << ", " 
                << torque_msg.torques[2] << "]");
    */
}

bool loadParameters(ros::NodeHandle& nh) {
    if (nh.getParam("/gamma_h", gamma_h)) {
        if (gamma_h.size() != 3) {
            ROS_ERROR("gamma_h should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded gamma_h from parameter server.");
    } else {
        ROS_ERROR("Failed to load gamma_h from parameter server.");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateRobustTorque_uridosc_simple");
    ros::NodeHandle nh;

    // Load gamma_h parameters from the YAML file
    if (!loadParameters(nh)) {
        ROS_ERROR("Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscribe to the relevant topics
    ros::Subscriber adaptive_params_sub = nh.subscribe("/uridosc_adaptive_params", 10, adaptiveParamsCallback); // gets pi,qi
    ros::Subscriber sliding_surface_sub = nh.subscribe("/ridosc_sliding_surface", 10, slidingSurfaceCallback);  // gets sigma

    // Publisher for calculated torque values
    torque_pub = nh.advertise<smm_control::FasmcTorques>("/ridosc_robust_term", 10);

    ros::Rate rate(UPDATE_ROBUST_TORQUE_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        calculateTorque();
        rate.sleep();
    }

    return 0;
}
