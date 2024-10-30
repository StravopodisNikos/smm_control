#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>
#include "smm_control/GetDynamics.h"  // Service header
#include "smm_control/timing.h"

// Globals for storing dynamic matrices
KDL::Chain kdl_chain;
std::unique_ptr<KDL::ChainDynParam> kdl_dynamics_solver;
KDL::JntArray joint_positions;
KDL::JntArray joint_velocities;
KDL::JntArray gravity_vector;

// Callback to update joint positions and velocities from /joint_current_state
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
    if (joint_state->position.size() != joint_positions.rows() || 
        joint_state->velocity.size() != joint_velocities.rows()) {
        ROS_WARN("Joint state size mismatch.");
        return;
    }

    for (size_t i = 0; i < joint_state->position.size(); ++i) {
        joint_positions(i) = joint_state->position[i];
        joint_velocities(i) = joint_state->velocity[i];
    }
}

// Service callback to provide requested Dynamic matrices
bool sendDynamics(smm_control::GetDynamics::Request &req, smm_control::GetDynamics::Response &res) {
    KDL::JntSpaceInertiaMatrix mass_matrix(kdl_chain.getNrOfJoints());
    KDL::JntArray gravity_torque(kdl_chain.getNrOfJoints());
    KDL::JntArray coriolis_torque(kdl_chain.getNrOfJoints());

    // Initialize the response arrays
    std::fill(res.Mass.begin(), res.Mass.end(), 0.0);
    std::fill(res.Coriolis.begin(), res.Coriolis.end(), 0.0);
    std::fill(res.Gravity.begin(), res.Gravity.end(), 0.0);

    // Mass matrix
    if (req.get_Mass) {
        kdl_dynamics_solver->JntToMass(joint_positions, mass_matrix);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                res.Mass[i * 3 + j] = mass_matrix(i, j);
            }
        }
    }

    // Coriolis matrix (approximated by Coriolis torque due to KDL limitations)
    if (req.get_Coriolis) {
        kdl_dynamics_solver->JntToCoriolis(joint_positions, joint_velocities, coriolis_torque);
        for (int i = 0; i < 3; ++i) {
            res.Coriolis[i * 3 + i] = coriolis_torque(i);
        }
    }

    // Gravity vector
    if (req.get_Gravity) {
        kdl_dynamics_solver->JntToGravity(joint_positions, gravity_torque);
        for (int i = 0; i < 3; ++i) {
            res.Gravity[i] = gravity_torque(i);
        }
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serverRobotDynamicsKDL");
    ros::NodeHandle nh;

    // Load URDF model
    urdf::Model urdf_model;
    if (!urdf_model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse URDF file.");
        return -1;
    }

    // Create KDL tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }

    // Define root and tip links
    std::string root_link = "world";  // Update as needed
    std::string tip_link = "force_sensor";  // Update as needed

    // Extract chain from KDL tree
    if (!kdl_tree.getChain(root_link, tip_link, kdl_chain)) {
        ROS_ERROR("Failed to extract KDL chain from %s to %s", root_link.c_str(), tip_link.c_str());
        return -1;
    }

    // Initialize dynamics solver
    KDL::Vector kdl_gravity(0, 0, -9.81);  // Set gravity vector
    kdl_dynamics_solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, kdl_gravity);

    // Initialize joint arrays
    joint_positions = KDL::JntArray(kdl_chain.getNrOfJoints());
    joint_velocities = KDL::JntArray(kdl_chain.getNrOfJoints());
    gravity_vector = KDL::JntArray(kdl_chain.getNrOfJoints());

    // Subscribe to joint states
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_current_state", 10, jointStateCallback);

    // Advertise the dynamics service
    ros::ServiceServer service = nh.advertiseService("GetRobotDynamics", sendDynamics);

    ros::Rate loop_rate(UPDATE_SERVER_DYNAMICS_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
