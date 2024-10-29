/*
 *  [22-10-24]   Node cpp file for [fasmc_simple] implementation of fasmc controller
 *               Fuzzy Adaptive Sliding Mode Control: Robust controller for SMM 
 *              - subscribes to gazebo joint states and publishes joint positions,
 *                velocities and accelerations to a single topic "/joint_current_state"
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/timing.h"

// Global variables for joint positions/ velocities/ accelerations
float q[DOF] = {0.0, 0.0, 0.0};
float dq[DOF] = {0.0, 0.0, 0.0};

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_state, 
                         ScrewsKinematics& smm_robot_kin_solver, 
                         ScrewsDynamics& smm_robot_dyn_solver) {
    // Check if joint state contains at least 3 positions
    if (joint_state->position.size() < DOF) {
        ROS_WARN("[updateJointState_fasmc_simple/jointStatesCallback] Expected 3 joint positions, received %zu", joint_state->position.size());
        return;
    }

    // Check if joint state contains at least 3 velocities
    if (joint_state->velocity.size() < DOF) {
        ROS_WARN("[updateJointState_fasmc_simple/jointStatesCallback] Expected 3 joint velocities, received %zu", joint_state->velocity.size());
        return;
    }

    // Acquire joint positions && velocities, indices differ due to gazebo state sequence
    q[0] = static_cast<float>(joint_state->position[2]);
    q[1] = static_cast<float>(joint_state->position[0]);
    q[2] = static_cast<float>(joint_state->position[1]);
    dq[0] = static_cast<float>(joint_state->velocity[2]);
    dq[1] = static_cast<float>(joint_state->velocity[0]);
    dq[2] = static_cast<float>(joint_state->velocity[1]);

    // Debug statement for updating joint states
    //ROS_INFO("[updateJointState_fasmc_simple/jointStatesCallback] Updating joint states with positions and velocities");

    // Update joint states in the solvers
    smm_robot_kin_solver.updateJointState(q, dq);
    smm_robot_dyn_solver.updateJointPos(q);
    smm_robot_dyn_solver.updateJointVel(dq);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "updateJointState_fasmc_simple");
    ros::NodeHandle nh;

    // Debug statement
    //ROS_INFO("[updateJointState_fasmc_simple] Initializing robot structure");

    // Initialize the robot structure
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    // Debug statement
    //ROS_INFO("[updateJointState_fasmc_simple] Initializing shared library");

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[updateJointState_fasmc_simple] Failed to initialize shared library.");
        return -1;
    }

    // Initialize the shared library for robot analytical solvers using screws
    //ROS_INFO("[updateJointState_fasmc_simple] Initialized Shared Library.");
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();  
    // ScrewsVisualization& smm_robot_viz_solver = my_shared_lib.get_screws_visualization_solver();  // -> not yet needed [26-9-24]

    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/smm_ros_gazebo/joint_states", 10, 
        boost::bind(jointStatesCallback, _1, boost::ref(smm_robot_kin_solver), boost::ref(smm_robot_dyn_solver))
    );

    // Publisher for fasmc current joint state
    ros::Publisher joint_current_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_current_state", 10);

    // Run in loop
    ros::Rate loop_rate(UPDATE_JOINT_STATE_RATE);
    while (ros::ok()) {

        sensor_msgs::JointState joint_current_state;
        joint_current_state.header.stamp = ros::Time::now();
        joint_current_state.name = {"joint1", "joint2", "joint3"};
        joint_current_state.position = {q[0], q[1], q[2]};
        joint_current_state.velocity = {dq[0], dq[1], dq[2]};
        joint_current_state_pub.publish(joint_current_state);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}