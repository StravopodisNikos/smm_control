#include <ros/ros.h>
#include "smm_screws/robot_shared.h"
#include <sensor_msgs/JointState.h>
#include <smm_control/CustomTcpState.h>
#include <smm_control/IdoscCurrent.h>
#include <smm_control/GetJacobians.h>
#include <smm_control/GetTcpStateTfNum.h>
#include <smm_control/IdoscError.h>
#include <smm_control/FasmcTorques.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <stdexcept>
#include "smm_control/timing.h"

// Globals
Eigen::Matrix3f _Jop;                                // Tool Jacobian
// Error state
Eigen::Vector3f _e = Eigen::Vector3f::Zero();     // TCP position error
Eigen::Vector3f _de = Eigen::Vector3f::Zero();    // TCP velocity error
// Desired state
Eigen::Vector3f _ddx_d = Eigen::Vector3f::Zero(); // Desired TCP acceleration loaded from yaml
// Current state
Eigen::Vector3f _ddx = Eigen::Vector3f::Zero();
Eigen::Vector3f _dx = Eigen::Vector3f::Zero();    // Current TCP velocity
Eigen::Vector3f _dq = Eigen::Vector3f::Zero();  // Current joint velocities
// Impedance torque vector
Eigen::Vector3f _u  = Eigen::Vector3f::Zero();  // Control effort vector
// msg with computed torques
ros::Publisher torque_pub;
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
Eigen::Vector3f _alpha_signal;                   // acceleration induced by impedance action

// Function to request acceleration
bool requestAcceleration(ros::NodeHandle& nh) {
    // Create a service client for the GetTcpVelAccelNum service
    ros::ServiceClient client = nh.serviceClient<smm_control::GetTcpStateTfNum>("GetTcpVelAccelNum");

    // Create the service request and response objects
    smm_control::GetTcpStateTfNum srv;
    srv.request.get_vel_num = false;  // No velocity required
    srv.request.get_accel_num = true; // Request acceleration

    // Call the service
    if (client.call(srv)) {
        // Assign acceleration data to Eigen::Vector3f
        _ddx(0) = srv.response.accel_num[0];
        _ddx(1) = srv.response.accel_num[1];
        _ddx(2) = srv.response.accel_num[2];

        return true;
    } else {
        ROS_ERROR("[updateImpedanceTorque/requestAcceleration] Failed to call service GetTcpVelAccelNum");
        return false;
    }
}

// Function to call the service and retrieve Jacobians
bool getOperationalJacobian(ros::NodeHandle& nh) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetJacobians>("GetOperationalJacobians");
    smm_control::GetJacobians srv;

    // Set flags in the request
    srv.request.get_op_jacobian = true;
    srv.request.get_inv_op_jacobian = false;
    srv.request.get_dt_op_jacobian = false;

    if (client.call(srv)) {
        // Convert the flat array back to 3x3 matrix
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                _Jop(i, j) = srv.response.op_jacobian[i * 3 + j];
            }
        }
        return true;
    } else {
        ROS_ERROR("[updateImpedanceTorque_ridosc_simple/getOperationalJacobian] Failed to call service GetOperationalJacobians.");
        return false;
    }
}

// Callback for joint states to retrieve joint accelerations
void desiredStateCallback(const smm_control::CustomTcpState::ConstPtr& msg) {
    if (msg->acceleration.size() >= 3) {
        _ddx_d << static_cast<float>(msg->acceleration[0]), 
                 static_cast<float>(msg->acceleration[1]), 
                 static_cast<float>(msg->acceleration[2]);
    } else {
        ROS_WARN("[updateImpedanceTorque/desiredStateCallback] Expected at least 3 joint accelerations, received %zu", msg->acceleration.size());
    }
}

void currentTcpStateCallback(const smm_control::IdoscCurrent::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Received a null IdoscError message.");
        return;
    }
    _dx << msg->twist_cur.linear.x,
           msg->twist_cur.linear.y,
           msg->twist_cur.linear.z;
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

void impedanceAccelerationCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[updateDynamicsTorque_ric_simple/impedanceAccelerationCallback] Received a null message on /imp_accel.");
        return;
    }
    // Assign the received data to the Eigen vector
    _alpha_signal << static_cast<float>(msg->x),
                     static_cast<float>(msg->y),
                     static_cast<float>(msg->z);
}

// Function to compute controller output
void computeImpedanceTorque(ros::NodeHandle& nh) {
    if (!getOperationalJacobian(nh)) {
        ROS_ERROR("Failed to retrieve operational space jacobian.");
        return;
    }
    /*if (!requestAcceleration(nh)) {
        ROS_ERROR("Failed to retrieve current arithmetic tcp acceleration.");
        return;
    }*/

    _u = _Jop.transpose() * ( _M_m * ( _ddx_d - _alpha_signal ) + _D_m * _de +  _K_m * _e);

    smm_control::FasmcTorques torque_msg;
    torque_msg.torques[0] = _u[0];
    torque_msg.torques[1] = _u[1];
    torque_msg.torques[2] = _u[2];

    torque_pub.publish(torque_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateImpedanceTorque");
    ros::NodeHandle nh;

    // Retrieve mode from the launch file argument
    std::string mode;
    if (!nh.getParam("impedance_mode", mode)) {
        ROS_ERROR("Failed to get 'mode' parameter. Please specify a mode in the launch file.");
        return -1;
    }

    // Load impedance params for given mode
    if (!loadImpedanceParams(nh,mode)) {
        ROS_ERROR("[updateImpedanceTorque] Failed to load parameters. Exiting.");
        return -1;
    }

    ros::Subscriber desired_state_sub = nh.subscribe("/tcp_desired_state", 10, desiredStateCallback);
    ros::Subscriber error_state_sub = nh.subscribe("/ridosc_error_state", 10, errorStateCallback);
    ros::Subscriber current_tcp_state_sub = nh.subscribe("/tcp_current_state", 10, currentTcpStateCallback);
    ros::Subscriber imp_accel_sub = nh.subscribe("/imp_accel", 10, impedanceAccelerationCallback);

    torque_pub = nh.advertise<smm_control::FasmcTorques>("/ric_imp_term", 10);

    ros::Rate loop_rate(UPDATE_IMPEDANCE_TORQUE_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        computeImpedanceTorque(nh);
        loop_rate.sleep();
    }

    return 0;
}
