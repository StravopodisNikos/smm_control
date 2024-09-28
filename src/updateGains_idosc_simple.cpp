#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <smm_control/IdoscGainConfig.h>  // Generated dynamic reconfigure header
#include <smm_control/IdoscGainMatrices.h>  // Custom message for kp and kd gains
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

Eigen::Matrix3f Kp;  // Diagonal matrix for proportional gains
Eigen::Matrix3f Kd;  // Diagonal matrix for derivative gains

// Limits for the gains from the YAML file
float kp1_min, kp1_max, kp2_min, kp2_max, kp3_min, kp3_max;
float kd1_min, kd1_max, kd2_min, kd2_max, kd3_min, kd3_max;

// Callback for dynamic reconfigure
void dynamicReconfigCallback(smm_control::IdoscGainConfig &config, uint32_t level) {
    // Update the diagonal matrix Kp with the reconfigured values
    Kp(0, 0) = config.kp1;
    Kp(1, 0) = 0.0; Kp(0, 1) = 0.0; Kp(2, 0) = 0.0; Kp(0, 2) = 0.0;
    Kp(1, 1) = config.kp2;
    Kp(2, 2) = config.kp3;

    // Update the diagonal matrix Kd with the reconfigured values
    Kd(0, 0) = config.kd1;
    Kd(1, 0) = 0.0; Kd(0, 1) = 0.0; Kd(2, 0) = 0.0; Kd(0, 2) = 0.0;
    Kd(1, 1) = config.kd2;
    Kd(2, 2) = config.kd3;

    ROS_INFO("Updated Kp matrix: \n%f %f %f\n%f %f %f\n%f %f %f",
             Kp(0, 0), Kp(0, 1), Kp(0, 2),
             Kp(1, 0), Kp(1, 1), Kp(1, 2),
             Kp(2, 0), Kp(2, 1), Kp(2, 2));

    ROS_INFO("Updated Kd matrix: \n%f %f %f\n%f %f %f\n%f %f %f",
             Kd(0, 0), Kd(0, 1), Kd(0, 2),
             Kd(1, 0), Kd(1, 1), Kd(1, 2),
             Kd(2, 0), Kd(2, 1), Kd(2, 2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "updateGains_idosc_simple");
    ros::NodeHandle nh;

    // Setup dynamic reconfigure server
    dynamic_reconfigure::Server<smm_control::IdoscGainConfig> server;
    dynamic_reconfigure::Server<smm_control::IdoscGainConfig>::CallbackType f;

    f = boost::bind(&dynamicReconfigCallback, _1, _2);
    server.setCallback(f);

    // Publisher for the gains
    ros::Publisher gains_pub = nh.advertise<smm_control::IdoscGainMatrices>("/idosc_gains", 10);

    // Set initial values of Kp and Kd to 0
    Kp.setZero();
    Kd.setZero();

    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
        // Create and populate the gains message
        smm_control::IdoscGainMatrices gains_msg;
        gains_msg.kp1 = Kp(0, 0);
        gains_msg.kp2 = Kp(1, 1);
        gains_msg.kp3 = Kp(2, 2);
        gains_msg.kd1 = Kd(0, 0);
        gains_msg.kd2 = Kd(1, 1);
        gains_msg.kd3 = Kd(2, 2);

        // Publish the gains message
        gains_pub.publish(gains_msg);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}