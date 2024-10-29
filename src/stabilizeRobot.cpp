#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <kdl/tree.hpp>      // For KDL::Tree
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>

// Globals
KDL::Chain kdl_chain;
std::unique_ptr<KDL::ChainDynParam> kdl_dynamics_solver;
KDL::JntArray current_positions;
KDL::JntArray gravity_torques;
Eigen::Vector3d gravity_vector(0, 0, -9.81);  // Standard gravity vector in z direction

// Publisher
ros::Publisher torque_pub;

// Callback to update joint positions
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() != current_positions.rows()) {
        ROS_ERROR("Received joint state does not match the robot's DOF.");
        return;
    }
    
    // Copy joint positions
    for (size_t i = 0; i < msg->position.size(); ++i) {
        current_positions(i) = msg->position[i];
    }
}

// Function to compute and publish gravity compensation torques
void computeGravityCompensation() {
    // Compute gravity torques
    kdl_dynamics_solver->JntToGravity(current_positions, gravity_torques);

    // Create and populate the torque message
    std_msgs::Float64MultiArray torque_msg;
    torque_msg.data.resize(gravity_torques.rows());
    for (size_t i = 0; i < gravity_torques.rows(); ++i) {
        torque_msg.data[i] = gravity_torques(i);
    }

    // Publish torques
    torque_pub.publish(torque_msg);
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "stabilizeRobot");
    ros::NodeHandle nh;

    // Load URDF model
    urdf::Model urdf_model;
    if (!urdf_model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse URDF file.");
        return -1;
    }

    // Declare the KDL tree
    KDL::Tree kdl_tree;
    
    // Parse the URDF model into a KDL tree
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }

    // Define the root and tip links for the KDL chain
    std::string root_link = "world";  // Change this to your robot's base link
    std::string tip_link = "force_sensor";  // Change this to your robot's end-effector link
    if (!kdl_tree.getChain(root_link, tip_link, kdl_chain)) {
        ROS_ERROR("Failed to extract KDL chain from %s to %s", root_link.c_str(), tip_link.c_str());
        return -1;
    }

    // Initialize the dynamic solver for gravity compensation
    KDL::Vector kdl_gravity(gravity_vector[0], gravity_vector[1], gravity_vector[2]);
    kdl_dynamics_solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, kdl_gravity);

    // Initialize joint arrays
    current_positions = KDL::JntArray(kdl_chain.getNrOfJoints());
    gravity_torques = KDL::JntArray(kdl_chain.getNrOfJoints());

    // Subscribers and publishers
    ros::Subscriber joint_state_sub = nh.subscribe("/smm_ros_gazebo/joint_states", 10, jointStateCallback);
    torque_pub = nh.advertise<std_msgs::Float64MultiArray>("/stabilize_robot/torques", 10);

    // Set loop rate for control
    ros::Rate loop_rate(10);  // 10 Hz

    // Main loop
    while (ros::ok()) {
        // Update and compute torques
        computeGravityCompensation();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
