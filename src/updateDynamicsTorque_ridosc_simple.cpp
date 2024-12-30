#include <ros/ros.h>
#include "smm_screws/robot_shared.h"
#include <smm_control/CustomTcpState.h>
#include <smm_control/IdoscCurrent.h>
#include <smm_control/GetOperationalSpaceDynamics.h>
#include <smm_control/IdoscError.h>
#include <smm_control/FasmcTorques.h>
#include <Eigen/Dense>
#include <vector>
#include "smm_control/timing.h"

// Globals
Eigen::Matrix3f _Lambda;                         // Mass Matrix @ TCP loaded from service
Eigen::Matrix3f _Gamma;                          // Coriolis Matrix @ TCP loaded from service 
Eigen::Vector3f _Fg;                             // Gravity Vector @ TCP loaded from service 
Eigen::Vector3f _u;                              // Control effort vector
std::vector<double> _k_p(3, 1.0);                // Initialized vector with P-gains
std::vector<double> _k_d(3, 1.0);                // Initialized vector with D-gains
std::vector<double> _d_c(3, 1.0);                // Initialized vector with damping coeffs  
std::vector<double> _f_c(3, 1.0);                // Initialized vector with friction coeffs
Eigen::Matrix3f _Kp = Eigen::Matrix3f::Identity(); // P gains
Eigen::Matrix3f _Kd = Eigen::Matrix3f::Identity(); // D gains
Eigen::Matrix3f _Damp = Eigen::Matrix3f::Identity(); // Damping coefficient matrix
Eigen::Matrix3f _Fric = Eigen::Matrix3f::Identity();// Friction coefficient matrix

// Error state
Eigen::Vector3f _e = Eigen::Vector3f::Zero();    // 
Eigen::Vector3f _de = Eigen::Vector3f::Zero();   // 

// Desired state
Eigen::Vector3f _ddx_d = Eigen::Vector3f::Zero(); // Desired TCP acceleration loaded from yaml

// Current state
Eigen::Vector3f _dx = Eigen::Vector3f::Zero();    // Current TCP velocity

ros::Publisher torque_pub;

// Function to call the service and retrieve Dynamic matrices
bool getDynamicsFromService(ros::NodeHandle& nh, bool get_LambdaMatrix, bool get_GammaMatrix, bool get_FgVector) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetOperationalSpaceDynamics>("GetOperationalSpaceDynamics");
    smm_control::GetOperationalSpaceDynamics srv;
 
    // Set flags in the request
    srv.request.get_Lambda = get_LambdaMatrix;
    srv.request.get_Gamma_OSD = get_GammaMatrix;
    srv.request.get_Fg = get_FgVector;

    if (client.call(srv)) {
        if (get_LambdaMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Lambda(i, j) = srv.response.Lambda[i * 3 + j];
                }
            }
            ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Lambda Matrix:\n" << _Lambda);
        }
        if (get_GammaMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Gamma(i, j) = srv.response.Gamma_OSD[i * 3 + j];
                }
            }
            ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Gamma Matrix:\n" << _Gamma);
        }
        if (get_FgVector) {
            for (int i = 0; i < 3; i++) {
                _Fg(i) = srv.response.Fg[i];
            }
            ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] TCP Gravity Vector:\n" << _Fg);
        }
        return true;        
    } else {
        ROS_ERROR("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Failed to call service GetOperationalSpaceDynamics.");
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
        ROS_WARN("[updateDynamicsTorque_ridosc_simple/desiredStateCallback] Expected at least 3 joint accelerations, received %zu", msg->acceleration.size());
    }
}

void currentStateCallback(const smm_control::IdoscCurrent::ConstPtr& msg) {
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

// Function to compute controller output
void computeJointEffort(ros::NodeHandle& nh) {
    if (!getDynamicsFromService(nh, true, true, true)) {
        ROS_ERROR("Failed to retrieve dynamic matrices.");
        return;
    }

    //_u = _M * (_ddqd + lambda_0.asDiagonal() * velocity_error) + (_C * _dq + _G) - ( _B * _dq ) - ( _F * _dq.array().sign().matrix() ); 

    smm_control::FasmcTorques torque_msg;
    torque_msg.torques[0] = _u[0];
    torque_msg.torques[1] = _u[1];
    torque_msg.torques[2] = _u[2];
    torque_pub.publish(torque_msg);

    ROS_INFO("[updateDynamicsTorque_ridosc_simple] Dynamic Model Torques: [%f, %f, %f]", torque_msg.torques[0],torque_msg.torques[0], torque_msg.torques[2]);
    
}

bool buildDiagonalMatrix(Eigen::Matrix3f* matrix_ptr, const std::vector<double>& diag_values) {
    // Check if the pointer is null
    if (matrix_ptr == nullptr) {
        std::cerr << "[updateDynamicsTorque_ridosc_simple/buildDiagonalMatrix] Error: Null pointer provided for matrix." << std::endl;
        return false;
    }

    // Check if the vector has exactly 3 elements
    if (diag_values.size() != 3) {
        std::cerr << "[updateDynamicsTorque_ridosc_simple/buildDiagonalMatrix] Error: Vector size must be exactly 3 for a 3x3 matrix." << std::endl;
        return false;
    }

    // Set the diagonal elements
    for (size_t i = 0; i < diag_values.size(); ++i) {
        (*matrix_ptr)(i, i) = static_cast<float>(diag_values[i]);
    }

    return true;
}

// Function to load parameters from the YAML file
bool loadParameters(ros::NodeHandle& nh) {
    if (nh.getParam("/_k_p", _k_p)) {
        if (_k_p.size() != 3) {
            ROS_ERROR("_k_p should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded _k_p from parameter server.");
        if (!buildDiagonalMatrix(&_Kp, _k_p)) {
            ROS_ERROR("Setting Kp matrix failed.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to load _k_p from parameter server.");
        return false;
    }
    if (nh.getParam("/_k_d", _k_d)) {
        if (_k_d.size() != 3) {
            ROS_ERROR("_k_d should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded _k_d from parameter server.");
        if (!buildDiagonalMatrix(&_Kd, _k_d)) {
            ROS_ERROR("Setting Kd matrix failed.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to load _k_d from parameter server.");
        return false;
    }
    if (nh.getParam("/_d_c", _d_c)) {
        if (_d_c.size() != 3) {
            ROS_ERROR("_d_c should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded _d_c from parameter server.");
        if (!buildDiagonalMatrix(&_Damp, _d_c)) {
            ROS_ERROR("Setting Dc matrix failed.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to load _d_c from parameter server.");
        return false;
    }
    if (nh.getParam("/_f_c", _f_c)) {
        if (_f_c.size() != 3) {
            ROS_ERROR("_f_c should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded _f_c from parameter server.");
        if (!buildDiagonalMatrix(&_Fric, _f_c)) {
            ROS_ERROR("Setting Fc matrix failed.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to load _f_c from parameter server.");
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateDynamicsTorque_ridosc_simple");
    ros::NodeHandle nh;

    if (!loadParameters(nh)) {
        ROS_ERROR("[updateDynamicsTorque_ridosc_simple] Failed to load parameters. Exiting.");
        return -1;
    }

    ros::Subscriber desired_state_sub = nh.subscribe("/tcp_desired_state", 10, desiredStateCallback);
    ros::Subscriber error_state_sub = nh.subscribe("/ridos_error_state", 10, errorStateCallback);
    ros::Subscriber current_state_sub = nh.subscribe("/tcp_current_state", 10, currentStateCallback);

    torque_pub = nh.advertise<smm_control::FasmcTorques>("/ridosc_dyn_term", 10);

    ros::Rate loop_rate(UPDATE_DYNAMICS_TORQUE_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        computeJointEffort(nh);
        loop_rate.sleep();
    }

    return 0;
}
