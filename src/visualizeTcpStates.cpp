#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/timing.h"

// Callback for combined joint states
void combinedJointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state, 
                                ScrewsKinematics& smm_robot_kin_solver, 
                                ScrewsVisualization& smm_robot_viz_solver) {
    if (joint_state->position.size() < 3 || joint_state->velocity.size() < 3) {
        ROS_WARN("Expected 3 joint positions and velocities, received %zu positions and %zu velocities", 
                  joint_state->position.size(), joint_state->velocity.size());
        return;
    }

    float q[3] = {static_cast<float>(joint_state->position[0]),
                  static_cast<float>(joint_state->position[1]),
                  static_cast<float>(joint_state->position[2])};

    float dq[3] = {static_cast<float>(joint_state->velocity[0]),
                   static_cast<float>(joint_state->velocity[1]),
                   static_cast<float>(joint_state->velocity[2])};

    // Debug statement
    //ROS_INFO("[visualizeTcpStates/combinedJointStateCallback] Updating joint states and velocities");

    // Update joint states
    smm_robot_kin_solver.updateJointState(q, dq);

    // Extract and publish the TCP spatial position
    Eigen::Vector3f tcp_pos = smm_robot_kin_solver.updatePositionTCP(q);
    //ROS_INFO("[visualizeTcpStates/combinedJointStateCallback] TCP Position: x: %f, y: %f, z: %f", tcp_pos.x(), tcp_pos.y(), tcp_pos.z());
    smm_robot_viz_solver.publishTCPpos(tcp_pos);

    // Extract and publish the Active Twists
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // Always run before use of SpatialJacobian_Tool_2() OR publishTwists().
    smm_robot_viz_solver.publishTwists(smm_robot_kin_solver.g_ptr);

    // Extract and publish the TCP spatial velocity
    smm_robot_kin_solver.SpatialJacobian_Tool_1(smm_robot_kin_solver.ptr2Jsp1);
    Eigen::Vector3f tcp_vel = smm_robot_kin_solver.updateSpatialVelocityTCP(q, dq);
    //ROS_INFO("[visualizeTcpStates/combinedJointStateCallback] TCP Velocity: x: %f, y: %f, z: %f", tcp_vel.x(), tcp_vel.y(), tcp_vel.z());
    smm_robot_viz_solver.publishTCPvel(tcp_pos,tcp_vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualizeTcpStates");
    ros::NodeHandle nh;

    // Debug statement
    //ROS_INFO("[visualizeTcpStates] Initializing robot structure");

    // Initialize the robot structure
    int str_digit_loc;
    if (!nh.getParam("/str_digit", str_digit_loc)) {
        ROS_ERROR("[visualizeTcpStates] Failed to get str_digit parameter.");
        return -1;
    }
    RobotAbstractBase* robot_ptr = nullptr;
    switch (str_digit_loc) {
        case 2:
            robot_ptr = new Structure2Pseudos();
            break;
        case 3:
            robot_ptr = new Structure3Pseudos();
            break;
        //case 4:
        //    robot_ptr = new Structure4Pseudos();
        //    break;
        default:
            ROS_ERROR("[visualizeTcpStates] Invalid str_digit value. Must be 2, 3, or 4.");
            return -1;
    }

    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[visualizeTcpStates] Failed to initialize shared library.");
        return -1;
    }

    // Initialize the shared library for robot analytical solvers using screws
    //ROS_INFO("[visualizeTcpStates] Initialized Shared Library.");
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsVisualization& smm_robot_viz_solver = my_shared_lib.get_screws_visualization_solver(); 

    // Subscriber to robot joint states
    ros::Subscriber combined_joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(combinedJointStateCallback, _1, boost::ref(smm_robot_kin_solver), boost::ref(smm_robot_viz_solver))
    );

    // Run in loop
    ros::Rate loop_rate(UPDATE_VISUAL_TCP_STATE_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}