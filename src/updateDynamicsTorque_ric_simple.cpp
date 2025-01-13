#include <ros/ros.h>
#include "smm_screws/robot_shared.h"
#include <sensor_msgs/JointState.h>
#include <smm_control/CustomTcpState.h>
#include <smm_control/IdoscCurrent.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <smm_control/GetImpedanceDynamics.h>
#include <smm_control/IdoscError.h>
#include <smm_control/FasmcTorques.h>
#include <Eigen/Dense>
#include <vector>
#include "smm_control/timing.h"

// Globals
Eigen::Matrix3f _Lambda;                         // Mass Matrix @ TCP loaded from service
Eigen::Matrix3f _Gamma;                          // Coriolis Matrix @ TCP loaded from service 
Eigen::Vector3f _GammaVector;
Eigen::Vector3f _Fg;                             // Gravity Vector @ TCP loaded from service 
Eigen::Vector3f _alpha_signal;                   // acceleration induced by impedance action
Eigen::Vector3f _u;                              // Control effort vector
Eigen::Vector3f _Ftask = Eigen::Vector3f::Zero();
std::vector<double> _d_c(3, 1.0);                // Initialized vector with damping coeffs  
std::vector<double> _f_c(3, 1.0);                // Initialized vector with friction coeffs
Eigen::Matrix3f _Damp = Eigen::Matrix3f::Identity(); // Damping coefficient matrix
Eigen::Matrix3f _Fric = Eigen::Matrix3f::Identity(); // Friction coefficient matrix
Eigen::Matrix3f _Jop;                                // Tool Jacobian
// Error state
Eigen::Vector3f _e = Eigen::Vector3f::Zero();     // TCP position error
Eigen::Vector3f _de = Eigen::Vector3f::Zero();    // TCP velocity error
// Desired state
Eigen::Vector3f _ddx_d = Eigen::Vector3f::Zero(); // Desired TCP acceleration loaded from yaml
// Current state
Eigen::Vector3f _dx = Eigen::Vector3f::Zero();    // Current TCP velocity
Eigen::Vector3f _dq = Eigen::Vector3f::Zero();  // Current joint velocities

ros::Publisher torque_pub;

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

// Function to call the service and retrieve Dynamic matrices
bool getDynamicsFromService(ros::NodeHandle& nh, bool get_JacobianMatrix, bool get_LambdaMatrix, bool get_GammaMatrix, bool get_FgVector) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetImpedanceDynamics>("GetImpedanceDynamics");
    smm_control::GetImpedanceDynamics srv;
 
    // Set flags in the request
    srv.request.get_op_jacobian = get_JacobianMatrix;
    srv.request.get_Lambda_imp = get_LambdaMatrix;
    srv.request.get_Gamma_imp = get_GammaMatrix;
    srv.request.get_Fg_imp = get_FgVector;

    if (client.call(srv)) {
        if (get_LambdaMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Lambda(i, j) = srv.response.Lambda_imp[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ric_simple/getDynamicsFromService] Lambda Matrix:\n" << _Lambda);
        }
        if (get_JacobianMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Jop(i, j) = srv.response.op_jacobian[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ric_simple/getDynamicsFromService] Operational Jacobian Matrix:\n" << _Jop);
        }
        if (get_GammaMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Gamma(i, j) = srv.response.Gamma_imp[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ric_simple/getDynamicsFromService] Gamma Matrix:\n" << _Gamma);
        }

        if (get_FgVector) {
            for (int i = 0; i < 3; i++) {
                _Fg(i) = srv.response.Fg_imp[i];
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ric_simple/getDynamicsFromService] TCP Gravity Vector:\n" << _Fg);
        }
        return true;        
    } else {
        ROS_ERROR("[updateDynamicsTorque_ric_simple/getDynamicsFromService] Failed to call service serverOperationalSpaceDynamics.");
        return false;
    }
}

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
/*void desiredStateCallback(const smm_control::CustomTcpState::ConstPtr& msg) {
    if (msg->acceleration.size() >= 3) {
        _ddx_d << static_cast<float>(msg->acceleration[0]), 
                 static_cast<float>(msg->acceleration[1]), 
                 static_cast<float>(msg->acceleration[2]);
    } else {
        ROS_WARN("[updateDynamicsTorque_ric_simple/desiredStateCallback] Expected at least 3 joint accelerations, received %zu", msg->acceleration.size());
    }
}*/

void currentTcpStateCallback(const smm_control::IdoscCurrent::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Received a null IdoscError message.");
        return;
    }
    _dx << msg->twist_cur.linear.x,
           msg->twist_cur.linear.y,
           msg->twist_cur.linear.z;
}

void currentJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->velocity.size() >= 3) {
        _dq << static_cast<float>(msg->velocity[0]), 
               static_cast<float>(msg->velocity[1]), 
               static_cast<float>(msg->velocity[2]);
    } else {
        ROS_WARN("[updateDynamicsTorque_ric_simple/currentJointStateCallback] Expected at least 3 joint velocities, received %zu", msg->velocity.size());
    }
}
/*
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
}*/

bool buildDiagonalMatrix(Eigen::Matrix3f* matrix_ptr, const std::vector<double>& diag_values) {
    // Check if the pointer is null
    if (matrix_ptr == nullptr) {
        std::cerr << "[updateDynamicsTorque_ric_simple/buildDiagonalMatrix] Error: Null pointer provided for matrix." << std::endl;
        return false;
    }

    // Check if the vector has exactly 3 elements
    if (diag_values.size() != 3) {
        std::cerr << "[updateDynamicsTorque_ric_simple/buildDiagonalMatrix] Error: Vector size must be exactly 3 for a 3x3 matrix." << std::endl;
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
    if (nh.getParam("/d_c", _d_c)) {
        if (_d_c.size() != 3) {
            ROS_ERROR("d_c should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded d_c from parameter server.");
        if (!buildDiagonalMatrix(&_Damp, _d_c)) {
            ROS_ERROR("Setting Dc matrix failed.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to load d_c from parameter server.");
        return false;
    }
    if (nh.getParam("/f_c", _f_c)) {
        if (_f_c.size() != 3) {
            ROS_ERROR("f_c should contain exactly 3 values.");
            return false;
        }
        ROS_INFO("Loaded f_c from parameter server.");
        if (!buildDiagonalMatrix(&_Fric, _f_c)) {
            ROS_ERROR("Setting Fc matrix failed.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to load f_c from parameter server.");
        return false;
    }
    return true;
}

// Function to compute controller output
void computeJointEffort(ros::NodeHandle& nh) {
    if (!getDynamicsFromService(nh, true, true, true, true)) {
        ROS_ERROR("Failed to retrieve operational space matrices.");
        return;
    }

    _u = _Jop.transpose() * ( _Lambda * _alpha_signal + _Gamma * _dx  + _Fg - _Ftask ) - ( _Damp * _dq ) - ( _Fric * _dq.array().sign().matrix());

    smm_control::FasmcTorques torque_msg;
    torque_msg.torques[0] = _u[0];
    torque_msg.torques[1] = _u[1];
    torque_msg.torques[2] = _u[2];

    torque_pub.publish(torque_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateDynamicsTorque_ric_simple");
    ros::NodeHandle nh;

    if (!loadParameters(nh)) {
        ROS_ERROR("[updateDynamicsTorque_ric_simple] Failed to load parameters. Exiting.");
        return -1;
    }

    //ros::Subscriber desired_state_sub = nh.subscribe("/tcp_desired_state", 10, desiredStateCallback);
    //ros::Subscriber error_state_sub = nh.subscribe("/ridosc_error_state", 10, errorStateCallback);
    ros::Subscriber current_tcp_state_sub = nh.subscribe("/tcp_current_state", 10, currentTcpStateCallback);
    ros::Subscriber current_joint_state_sub = nh.subscribe("/joint_current_state", 10, currentJointStateCallback);
    ros::Subscriber ext_force_torque_sub = nh.subscribe("/force_torque_wrench", 10, extForceTorqueCallback);
    ros::Subscriber imp_accel_sub = nh.subscribe("/imp_accel", 10, impedanceAccelerationCallback);

    torque_pub = nh.advertise<smm_control::FasmcTorques>("/ric_dyn_term", 10);

    ros::Rate loop_rate(UPDATE_DYNAMICS_TORQUE_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        computeJointEffort(nh);
        loop_rate.sleep();
    }

    return 0;
}
