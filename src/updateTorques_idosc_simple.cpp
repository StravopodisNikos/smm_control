#include <ros/ros.h>
#include "smm_screws/robot_shared.h"
#include <sensor_msgs/JointState.h>
#include <smm_control/IdoscControlOutput.h>
#include <smm_control/GetDynamics.h>
#include <smm_control/IdoscTorques.h>
#include <Eigen/Dense>
#include <vector>

// Controller torques vector
Eigen::Matrix<float, DOF, 1> _u;
// Controller output vector
Eigen::Matrix<float, DOF, 1> _y;
// Joint velocities
Eigen::Matrix<float, DOF, 1> _dq;
// The Dynamic Matrices
Eigen::Matrix3f _M;
Eigen::Matrix3f _C;
Eigen::Matrix<float, DOF, 1> _G;

// Function to call the service and retrieve Dynamic matrices
bool getDynamicsFromService(ros::NodeHandle& nh, bool get_M, bool get_Cor, bool get_Grav) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetDynamics>("GetRobotDynamics");
    smm_control::GetDynamics srv;

    // Set flags in the request
    srv.request.get_Mass = get_M;
    srv.request.get_Coriolis = get_Cor;
    srv.request.get_Gravity = get_Grav;

    if (client.call(srv)) {
        if (get_M) {
            // Convert the flat array back to 3x3 matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _M(i, j) = srv.response.Mass[i * 3 + j];
                }
            }
            //ROS_INFO("[updateTorques_idosc_simple/getDynamicsFromService] Inverse Operational Jacobian retrieved.");
        }
        if (get_Cor) {
            // Convert the flat array back to 3x3 matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _C(i, j) = srv.response.Coriolis[i * 3 + j];
                }
            }
            //ROS_INFO("[updateTorques_idosc_simple/getDynamicsFromService] Time Derivative of Operational Jacobian retrieved.");
        }
        if (get_Grav) {
            for (int i = 0; i < 3; i++) {
                _G(i) = srv.response.Gravity[i];
            }
        }
        return true;        
    } else {
        ROS_ERROR("[updateTorques_idosc_simple/getDynamicsFromService] Failed to call service GetDynamics.");
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
        ROS_WARN("[updateTorques_idosc_simple/jointStateCallback] Expected at least 3 joint velocities, received %zu", msg->velocity.size());
    }
}

// Callback for idosc_error_state to retrieve TCP position and velocity errors
void controlOutputCallback(const smm_control::IdoscControlOutput::ConstPtr& msg) {
    // Set TCP position error
    _y[0] = msg->control_output[0];
    _y[1] = msg->control_output[1];
    _y[2] = msg->control_output[2];
}

// Function to compute controller output
void computeJointEffort(ros::NodeHandle& nh) {
    
    getDynamicsFromService(nh, true, true, true);

    //ROS_INFO("[updateTorques_idosc_simple/computeJointEffort] Mass matrix:\n%f %f %f\n%f %f %f\n%f %f %f",
    //          _M(0, 0), _M(0, 1), _M(0, 2),
    //          _M(1, 0), _M(1, 1), _M(1, 2),
    //          _M(2, 0), _M(2, 1), _M(2, 2));
    
    _u = _M * _y + ( _C * _dq + _G );

    ROS_INFO(" [updateTorques_idosc_simple/computeJointEffort] u: %f, %f, %f", _u[0], _u[1], _u[2]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateTorques_idosc_simple");
    ros::NodeHandle nh;

    // Subscriber for joint current state (joint velocities)
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_current_state", 10, jointStateCallback);

    // Subscriber for the controller vector output
    ros::Subscriber controller_sub = nh.subscribe("/idosc_control_output", 10, controlOutputCallback);

    // Publisher for the joint effort controllers
    ros::Publisher torques_pub = nh.advertise<smm_control::IdoscTorques>("/idosc_torques", 10);

    ros::Rate loop_rate(10);  // Run at 10 Hz

    while (ros::ok()) {
        // Compute the control output
        computeJointEffort(nh);

        // Create and populate the control output message
        smm_control::IdoscTorques robot_torques_msg;
        robot_torques_msg.robot_torques[0] = _u[0];
        robot_torques_msg.robot_torques[1] = _u[1];
        robot_torques_msg.robot_torques[2] = _u[2];

        // Publish the control output
        torques_pub.publish(robot_torques_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}