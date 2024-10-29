#include <ros/ros.h>
#include "smm_screws/robot_shared.h"
#include <smm_control/CustomJointState.h>
#include <sensor_msgs/JointState.h>
#include <smm_control/GetDynamics.h>
#include <smm_control/FasmcError.h>
#include <smm_control/FasmcTorques.h>
#include <Eigen/Dense>
#include <vector>
#include "smm_control/timing.h"

// Globals
Eigen::Matrix3f _M;
Eigen::Matrix3f _C;
Eigen::Vector3f _G;
Eigen::Vector3f _u;  // Control effort vector

// Error state
Eigen::Vector3f velocity_error = Eigen::Vector3f::Zero();
// Adaptive parameter
Eigen::Vector3f lambda_0 = Eigen::Vector3f::Constant(0.01);  // Default, overwritten by YAML

// Desired state
Eigen::Vector3f _ddqd = Eigen::Vector3f::Zero();  // Desired joint accelerations

// Current state
Eigen::Vector3f _dq = Eigen::Vector3f::Zero();  // Current joint velocities

ros::Publisher torque_pub;

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
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _M(i, j) = srv.response.Mass[i * 3 + j];
                }
            }
        }
        if (get_Cor) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _C(i, j) = srv.response.Coriolis[i * 3 + j];
                }
            }
        }
        if (get_Grav) {
            for (int i = 0; i < 3; i++) {
                _G(i) = srv.response.Gravity[i];
            }
        }
        return true;        
    } else {
        ROS_ERROR("[updateDynamicsTorque_fasmc_simple/getDynamicsFromService] Failed to call service GetDynamics.");
        return false;
    }
}

// Callback for joint states to retrieve joint accelerations
void desiredStateCallback(const smm_control::CustomJointState::ConstPtr& msg) {
    if (msg->acceleration.size() >= 3) {
        _ddqd << static_cast<float>(msg->acceleration[0]), 
               static_cast<float>(msg->acceleration[1]), 
               static_cast<float>(msg->acceleration[2]);
    } else {
        ROS_WARN("[updateDynamicsTorque_fasmc_simple/desiredStateCallback] Expected at least 3 joint accelerations, received %zu", msg->acceleration.size());
    }
}

void currentStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->velocity.size() >= 3) {
        _dq << static_cast<float>(msg->velocity[0]), 
               static_cast<float>(msg->velocity[1]), 
               static_cast<float>(msg->velocity[2]);
    } else {
        ROS_WARN("[updateDynamicsTorque_fasmc_simple/currentStateCallback] Expected at least 3 joint velocities, received %zu", msg->velocity.size());
    }
}

// Callback for /fasmc_error_state
void errorStateCallback(const smm_control::FasmcError::ConstPtr& msg) {
    if (msg->velocity_error.size() >= 3) {
        velocity_error << msg->velocity_error[0], msg->velocity_error[1], msg->velocity_error[2];
    } else {
        ROS_WARN("[updateDynamicsTorque_fasmc_simple/errorStateCallback] Expected at least 3 velocity errors, received %zu", msg->velocity_error.size());
    }
}

// Function to compute controller output
void computeJointEffort(ros::NodeHandle& nh) {
    if (!getDynamicsFromService(nh, true, true, true)) {
        ROS_ERROR("Failed to retrieve dynamic matrices.");
        return;
    }

    _u = _M * (_ddqd - lambda_0.asDiagonal() * velocity_error) + (_C * _dq + _G);

    smm_control::FasmcTorques torque_msg;
    torque_msg.torques[0] = _u[0];
    torque_msg.torques[0] = _u[1];
    torque_msg.torques[2] = _u[2];
    torque_pub.publish(torque_msg);

    ROS_INFO("[updateDynamicsTorque_fasmc_simple] Dynamic Model Torques: [%f, %f, %f]", torque_msg.torques[0],torque_msg.torques[0], torque_msg.torques[2]);
    
}

// Function to load parameters from the YAML file
bool loadParameters(ros::NodeHandle& nh) {
    std::vector<double> lambda_values;
    if (nh.getParam("/lambda_0", lambda_values)) {
        if (lambda_values.size() == 3) {
            for (int i = 0; i < 3; ++i) {
                lambda_0[i] = static_cast<float>(lambda_values[i]);
            }
            ROS_INFO("Loaded lambda_0 from parameter server.");
            return true;
        } else {
            ROS_ERROR("lambda_0 should contain exactly 3 values.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to load lambda_0 from parameter server.");
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateDynamicsTorque_fasmc_simple");
    ros::NodeHandle nh;

    // Load lambda_0 parameters from the YAML file
    if (!loadParameters(nh)) {
        ROS_ERROR("[updateDynamicsTorque_fasmc_simple] Failed to load parameters. Exiting.");
        return -1;
    }

    ros::Subscriber desired_state_sub = nh.subscribe("/joint_desired_state", 10, desiredStateCallback);
    ros::Subscriber error_state_sub = nh.subscribe("/fasmc_error_state", 10, errorStateCallback);
    ros::Subscriber current_state_sub = nh.subscribe("/joint_current_state", 10, currentStateCallback);

    torque_pub = nh.advertise<smm_control::FasmcTorques>("/fasmc_dyn_term", 10);

    ros::Rate loop_rate(UPDATE_DYNAMICS_TORQUE_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        computeJointEffort(nh);
        loop_rate.sleep();
    }

    return 0;
}
