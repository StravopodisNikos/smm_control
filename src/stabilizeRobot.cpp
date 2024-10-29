#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <smm_control/CustomJointState.h>  // Message for target position
#include <smm_control/RobotTorques.h>      // Custom message type for torques
#include <kdl/tree.hpp>      
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>

// Globals
KDL::Chain kdl_chain;
std::unique_ptr<KDL::ChainDynParam> kdl_dynamics_solver;
KDL::JntArray current_positions;
KDL::JntArray target_positions;  // Target joint positions from /joint_desired_state
KDL::JntArray gravity_torques;
Eigen::Vector3d gravity_vector(0, 0, -9.81);  // Standard gravity vector in z direction

// Publisher
ros::Publisher torque_pub;

// Callback to update the target joint positions from /joint_desired_state
void desiredStateCallback(const smm_control::CustomJointState::ConstPtr& msg) {
    if (msg->position.size() != target_positions.rows()) {
        ROS_ERROR("Received joint desired state does not match the robot's DOF.");
        return;
    }
    
    // Copy target joint positions
    for (size_t i = 0; i < msg->position.size(); ++i) {
        target_positions(i) = msg->position[i];
    }
}

// Callback to update the current joint positions from /joint_states
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() != current_positions.rows()) {
        ROS_ERROR("Received joint state does not match the robot's DOF.");
        return;
    }
    
    // Copy current joint positions
    for (size_t i = 0; i < msg->position.size(); ++i) {
        current_positions(i) = msg->position[i];
    }
}

// Function to compute and publish gravity compensation torques based on target positions
void computeGravityCompensation() {
    // Compute gravity torques needed to hold the target position
    kdl_dynamics_solver->JntToGravity(target_positions, gravity_torques);

    // Create and populate the torque message
    smm_control::RobotTorques torque_msg;
    for (size_t i = 0; i < gravity_torques.rows(); ++i) {
        torque_msg.torques[i] = 0.1 * gravity_torques(i);
    }

    // Publish torques
    torque_pub.publish(torque_msg);
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "stabilizeRobotCommand");
    ros::NodeHandle nh;

    // Load URDF model
    urdf::Model urdf_model;
    if (!urdf_model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse URDF file.");
        return -1;
    }

    // Declare the KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }

    std::string root_link = "world";
    std::string tip_link = "force_sensor";
    if (!kdl_tree.getChain(root_link, tip_link, kdl_chain)) {
        ROS_ERROR("Failed to extract KDL chain from %s to %s", root_link.c_str(), tip_link.c_str());
        return -1;
    }

    KDL::Vector kdl_gravity(gravity_vector[0], gravity_vector[1], gravity_vector[2]);
    kdl_dynamics_solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, kdl_gravity);

    current_positions = KDL::JntArray(kdl_chain.getNrOfJoints());
    target_positions = KDL::JntArray(kdl_chain.getNrOfJoints());
    gravity_torques = KDL::JntArray(kdl_chain.getNrOfJoints());

    ros::Subscriber joint_state_sub = nh.subscribe("/smm_ros_gazebo/joint_states", 10, jointStateCallback);
    ros::Subscriber desired_state_sub = nh.subscribe("/joint_desired_state", 10, desiredStateCallback);
    torque_pub = nh.advertise<smm_control::RobotTorques>("/robot_torque_command", 10);

    // Wait for joint states topic
    ROS_INFO("Waiting for /smm_ros_gazebo/joint_states...");
    ros::topic::waitForMessage<sensor_msgs::JointState>("/smm_ros_gazebo/joint_states");
    ROS_INFO("/smm_ros_gazebo/joint_states is now available.");

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        computeGravityCompensation();  // Compute and publish torques to hold position
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
