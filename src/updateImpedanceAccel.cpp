#include <ros/ros.h>
#include <smm_control/CustomTcpState.h>
#include <smm_control/IdoscError.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <stdexcept>
#include "smm_control/timing.h"

// Globals
// Error state
Eigen::Vector3f _e = Eigen::Vector3f::Zero();           // TCP position error
Eigen::Vector3f _de = Eigen::Vector3f::Zero();          // TCP velocity error
// Desired state
Eigen::Vector3f _ddx_d = Eigen::Vector3f::Zero();       // Desired TCP acceleration loaded from yaml
// Impedance acceleration vector
Eigen::Vector3f _alpha_imp= Eigen::Vector3f::Zero();    // Acceleration vector
// msg with computed torques
ros::Publisher imp_accel_pub;
// External forces applied @TCP
Eigen::Vector3f _Ftask = Eigen::Vector3f::Zero();
// Struct to store impedance parameters
struct ImpedanceParams {
    Eigen::Vector3f m_m;
    Eigen::Vector3f d_m;
    Eigen::Vector3f k_m;
} params;
// The Impedance matrices
Eigen::Matrix3f _M_m;
Eigen::Matrix3f _D_m;
Eigen::Matrix3f _K_m;

// Callback function for the external force/torque topic
void extForceTorqueCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    // Extract the force components and store them in the Eigen::Vector3f
    _Ftask << static_cast<float>(msg->wrench.force.x),
              static_cast<float>(msg->wrench.force.y),
              static_cast<float>(msg->wrench.force.z);

    // Debug print
    //ROS_INFO_STREAM("External Force: [" << external_force.transpose() << "]");
}

// Callback for joint states to retrieve joint accelerations
void desiredStateCallback(const smm_control::CustomTcpState::ConstPtr& msg) {
    if (msg->acceleration.size() >= 3) {
        _ddx_d << static_cast<float>(msg->acceleration[0]), 
                 static_cast<float>(msg->acceleration[1]), 
                 static_cast<float>(msg->acceleration[2]);
    } else {
        ROS_WARN("[updateImpedanceAccel/desiredStateCallback] Expected at least 3 joint accelerations, received %zu", msg->acceleration.size());
    }
}

void errorStateCallback(const smm_control::IdoscError::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Received a null IdoscError message.");
        return;
    }
    _e << msg->pose_error.position.x,
          msg->pose_error.position.y,
          msg->pose_error.position.z;
    _de << msg->twist_error.linear.x,
           msg->twist_error.linear.y,
           msg->twist_error.linear.z;
}

// Function to load the parameters for a specific mode
bool loadImpedanceParams(const ros::NodeHandle& nh, const std::string& mode) {
    // Base YAML path
    std::string base_path = "parameters/" + mode;

    std::vector<float> m_m, d_m, k_m;

    if (!nh.getParam(base_path + "/m_m", m_m) || m_m.size() != 3) {
        ROS_ERROR_STREAM("Failed to load m_m for mode: " << mode);
        return false;
    }
    if (!nh.getParam(base_path + "/d_m", d_m) || d_m.size() != 3) {
        ROS_ERROR_STREAM("Failed to load d_m for mode: " << mode);
        return false;
    }
    if (!nh.getParam(base_path + "/k_m", k_m) || k_m.size() != 3) {
        ROS_ERROR_STREAM("Failed to load k_m for mode: " << mode);
        return false;
    }

    // Convert to Eigen::Vector3f
    params.m_m = Eigen::Vector3f(m_m[0], m_m[1], m_m[2]);
    params.d_m = Eigen::Vector3f(d_m[0], d_m[1], d_m[2]);
    params.k_m = Eigen::Vector3f(k_m[0], k_m[1], k_m[2]);

    _M_m = params.m_m.asDiagonal();
    _D_m = params.d_m.asDiagonal();
    _K_m = params.k_m.asDiagonal();

    ROS_INFO_STREAM("Loaded impedance parameters for mode: " << mode);
    return true;
}

// Function to compute controller output
void computeImpedanceAccel(ros::NodeHandle& nh) {
    geometry_msgs::Vector3 msg;

    _alpha_imp = _ddx_d +  _M_m.inverse() * (_D_m * _de +  _K_m * _e + _Ftask);
    
    msg.x = static_cast<double>(_alpha_imp.x());
    msg.y = static_cast<double>(_alpha_imp.y());
    msg.z = static_cast<double>(_alpha_imp.z());

    imp_accel_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateImpedanceAccel");
    ros::NodeHandle nh;

    // Retrieve mode from the launch file argument
    std::string mode;
    if (!nh.getParam("impedance_mode", mode)) {
        ROS_ERROR("Failed to get 'mode' parameter. Please specify a mode in the launch file.");
        return -1;
    }

    // Load impedance params for given mode
    if (!loadImpedanceParams(nh,mode)) {
        ROS_ERROR("[updateImpedanceAccel] Failed to load parameters. Exiting.");
        return -1;
    }

    ros::Subscriber desired_state_sub = nh.subscribe("/tcp_desired_state", 10, desiredStateCallback);
    ros::Subscriber error_state_sub = nh.subscribe("/ridosc_error_state", 10, errorStateCallback);

    imp_accel_pub = nh.advertise<geometry_msgs::Vector3>("/imp_accel", 10);

    ros::Rate loop_rate(UPDATE_IMPEDANCE_ACCEL_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        computeImpedanceAccel(nh);
        loop_rate.sleep();
    }

    return 0;
}
