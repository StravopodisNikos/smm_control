#include <ros/ros.h>
#include <smm_control/FasmcTorques.h>
#include <smm_control/RobotTorques.h>
#include "smm_control/timing.h"

// Global variables to store the incoming torques
smm_control::FasmcTorques robust_term;
smm_control::FasmcTorques dyn_term;
smm_control::FasmcTorques imp_term;

// Flags to check if messages have been received
bool received_robust_term = false;
bool received_dyn_term = false;
bool received_imp_term = false;

// Callback for /ridosc_robust_term
void robustTermCallback(const smm_control::FasmcTorques::ConstPtr& msg) {
    robust_term = *msg;
    received_robust_term = true;
}

// Callback for /ric_dyn_term
void dynTermCallback(const smm_control::FasmcTorques::ConstPtr& msg) {
    dyn_term = *msg;
    received_dyn_term = true;
}

// Callback for /ric_imp_term
void impTermCallback(const smm_control::FasmcTorques::ConstPtr& msg) {
    imp_term = *msg;
    received_imp_term = true;
}

// Function to compute and publish the combined torques
void publishCombinedTorque(ros::Publisher& torque_pub) {
    if (received_robust_term && received_dyn_term && received_imp_term) {
        smm_control::RobotTorques combined_torque_msg;

        // Add the torque values from robust_term and dyn_term
        for (int i = 0; i < 3; ++i) {
            combined_torque_msg.torques[i] = robust_term.torques[i] + dyn_term.torques[i] + imp_term.torques[i];
        }

        // Publish the combined torques
        torque_pub.publish(combined_torque_msg);
    } else {
        ROS_WARN("[updateTorqueCommand_ric_simple/publishCombinedTorque] Waiting for both /ridosc_robust_term and /ridosc_dyn_term messages.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateTorqueCommand_ric_simple");
    ros::NodeHandle nh;

    // Subscribers for the torque terms
    ros::Subscriber robust_term_sub = nh.subscribe("/ridosc_robust_term", 10, robustTermCallback);
    ros::Subscriber dyn_term_sub = nh.subscribe("/ric_dyn_term", 10, dynTermCallback);
    ros::Subscriber imp_term_sub = nh.subscribe("/ric_imp_term", 10, impTermCallback);

    // Publisher for the combined torque command
    ros::Publisher torque_pub = nh.advertise<smm_control::RobotTorques>("/robot_torque_command", 10);

    ros::Rate loop_rate(UPDATE_TORQUE_COMMAND_RATE);

    while (ros::ok()) {
        // Compute and publish the combined torque command
        publishCombinedTorque(torque_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
