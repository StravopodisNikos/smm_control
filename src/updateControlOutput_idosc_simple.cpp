#include <ros/ros.h>
#include "smm_screws/robot_shared.h"
#include <sensor_msgs/JointState.h>
#include <smm_control/IdoscGainMatrices.h>
#include <smm_control/IdoscError.h> 
#include <smm_control/IdoscControlOutput.h>
#include <smm_control/GetJacobians.h>
#include <Eigen/Dense>
#include <vector>

// Controller output vector
Eigen::Matrix<float, DOF, 1> _y;
// IDOSC error state vectors
Eigen::Matrix<float, DOF, 1> _x1; // tcp vel error
Eigen::Matrix<float, DOF, 1> _x2; // tcp pos erros
// Eigen matrices for Kp and Kd
Eigen::Matrix3f Kp = Eigen::Matrix3f::Zero();
Eigen::Matrix3f Kd = Eigen::Matrix3f::Zero();
// Joint velocities
Eigen::Matrix<float, DOF, 1> _dq;
// TCP position and velocity errors
Eigen::Vector3f tcp_pos_error;
Eigen::Vector3f tcp_vel_error;
// The Jacobian and other matrices
Eigen::Matrix3f _iJop;   // Inverse Jacobian
Eigen::Matrix3f _dtJop;  // Time derivative of Jacobian

// Function to call the service and retrieve Jacobians
bool getJacobiansFromService(ros::NodeHandle& nh, bool get_op, bool get_inv_op, bool get_dt_op) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetJacobians>("GetOperationalJacobians");
    smm_control::GetJacobians srv;

    // Set flags in the request
    srv.request.get_op_jacobian = get_op;
    srv.request.get_inv_op_jacobian = get_inv_op;
    srv.request.get_dt_op_jacobian = get_dt_op;

    if (client.call(srv)) {
        if (get_inv_op) {
            // Convert the flat array back to 3x3 matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _iJop(i, j) = srv.response.inv_op_jacobian[i * 3 + j];
                }
            }
            ROS_INFO("[updateControlOutput_idosc_simple/getJacobiansFromService] Inverse Operational Jacobian retrieved.");
        }
        if (get_dt_op) {
            // Convert the flat array back to 3x3 matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _dtJop(i, j) = srv.response.dt_op_jacobian[i * 3 + j];
                }
            }
            ROS_INFO("[updateControlOutput_idosc_simple/getJacobiansFromService] Time Derivative of Operational Jacobian retrieved.");
        }
        return true;
    } else {
        ROS_ERROR("[updateControlOutput_idosc_simple/getJacobiansFromService] Failed to call service GetJacobians.");
        return false;
    }
}

// Callback for joint states to retrieve joint velocities
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->velocity.size() >= DOF) {
        // Store joint velocities in _dq (Eigen::Matrix<float, DOF, 1>)
        _dq << static_cast<float>(msg->velocity[0]), 
               static_cast<float>(msg->velocity[1]), 
               static_cast<float>(msg->velocity[2]);
    } else {
        ROS_WARN("[updateControlOutput_idosc_simple/jointStateCallback] Expected at least 3 joint velocities, received %zu", msg->velocity.size());
    }
}

// Callback for idosc_gains to retrieve Kp and Kd values
void gainsCallback(const smm_control::IdoscGainMatrices::ConstPtr& msg) {
    // Set diagonal values for Kp
    Kp(0, 0) = msg->kp1;
    Kp(1, 1) = msg->kp2;
    Kp(2, 2) = msg->kp3;
    // Set diagonal values for Kd
    Kd(0, 0) = msg->kd1;
    Kd(1, 1) = msg->kd2;
    Kd(2, 2) = msg->kd3;
}

// Callback for idosc_error_state to retrieve TCP position and velocity errors
void errorStateCallback(const smm_control::IdoscError::ConstPtr& msg) {
    // Set TCP position error
    tcp_pos_error << msg->pose_error.position.x, 
                     msg->pose_error.position.y, 
                     msg->pose_error.position.z;

    // Set TCP velocity error
    tcp_vel_error << msg->twist_error.linear.x, 
                     msg->twist_error.linear.y, 
                     msg->twist_error.linear.z;

    // Store error in matrices for control computation
    _x1 << tcp_vel_error[0], tcp_vel_error[1], tcp_vel_error[2];  // Velocity error
    _x2 << tcp_pos_error[0], tcp_pos_error[1], tcp_pos_error[2];  // Position error
}

// Function to compute controller output
void computeControlOutput(ros::NodeHandle& nh) {
    
    getJacobiansFromService(nh, false, true, true);

    ROS_INFO("[updateControlOutput_idosc_simple/computeControlOutput] Inverse Jacobian:\n%f %f %f\n%f %f %f\n%f %f %f",
              _iJop(0, 0), _iJop(0, 1), _iJop(0, 2),
              _iJop(1, 0), _iJop(1, 1), _iJop(1, 2),
              _iJop(2, 0), _iJop(2, 1), _iJop(2, 2));

    ROS_INFO("[updateControlOutput_idosc_simple/computeControlOutput] Time Derivative of Jacobian:\n%f %f %f\n%f %f %f\n%f %f %f",
              _dtJop(0, 0), _dtJop(0, 1), _dtJop(0, 2),
              _dtJop(1, 0), _dtJop(1, 1), _dtJop(1, 2),
              _dtJop(2, 0), _dtJop(2, 1), _dtJop(2, 2));
    
    // Compute controller output: _y = _iJop * ( _Kd * _x1 + _Kp * _x2 - _dtJop * _dq);
    _y = _iJop * (Kd * _x1 + Kp * _x2 - _dtJop * _dq);

    ROS_INFO(" [updateControlOutput_idosc_simple/computeControlOutput] y: %f, %f, %f", _y[0], _y[1], _y[2]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateControlOutput_idosc_simple");
    ros::NodeHandle nh;

    // Subscriber for joint current state (joint velocities)
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_current_state", 10, jointStateCallback);

    // Subscriber for gains (Kp and Kd values)
    ros::Subscriber gains_sub = nh.subscribe("/idosc_gains", 10, gainsCallback);

    // Subscriber for error state (TCP position and velocity errors)
    ros::Subscriber error_state_sub = nh.subscribe("/idosc_error_state", 10, errorStateCallback);

    // Publisher for the control output
    ros::Publisher control_output_pub = nh.advertise<smm_control::IdoscControlOutput>("/idosc_control_output", 10);

    ros::Rate loop_rate(10);  // Run at 10 Hz

    // Dummy initializaation
    _y.setZero();
    _x1.setZero();
    _x2.setZero();

    while (ros::ok()) {
        // Compute the control output
        computeControlOutput(nh);

        // Create and populate the control output message
        smm_control::IdoscControlOutput control_output_msg;
        control_output_msg.control_output[0] = _y[0];
        control_output_msg.control_output[1] = _y[1];
        control_output_msg.control_output[2] = _y[2];

        // Publish the control output
        control_output_pub.publish(control_output_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}