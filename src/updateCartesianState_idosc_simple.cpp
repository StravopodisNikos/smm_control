#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/IdoscCurrent.h"  

Eigen::Vector3f tcp_vel; // dxe X(1:3)
Eigen::Vector3f tcp_pos; // xe  X(4:6)

// Callback for combined joint states
void idosc_simple_JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state, 
                                ScrewsKinematics& smm_robot_kin_solver, 
                                ScrewsVisualization& smm_robot_viz_solver) {
    if (joint_state->position.size() < DOF || joint_state->velocity.size() < DOF) {
        ROS_WARN("[updateCartesianState_idosc_simple/idosc_simple_JointStateCallback] Expected 3 joint positions and velocities, received %zu positions and %zu velocities", 
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
    //ROS_INFO("[updateCartesianState_idosc_simple/idosc_simple_JointStateCallback] Updating joint states and velocities");

    // Update joint states
    smm_robot_kin_solver.updateJointState(q, dq);

    // Extract and publish the TCP spatial position [tcp_pos = xe]
    tcp_pos = smm_robot_kin_solver.updatePositionTCP(q);
    ROS_INFO("[idosc_simple_JointStateCallback] TCP Position: x: %f, y: %f, z: %f", tcp_pos.x(), tcp_pos.y(), tcp_pos.z());
    //smm_robot_viz_solver.publishTCPpos(tcp_pos);

    // Extract and publish the Active Twists
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // Always run before use of SpatialJacobian_Tool_2() OR publishTwists().
    //smm_robot_viz_solver.publishTwists(smm_robot_kin_solver.g_ptr);

    // Extract and publish the TCP spatial velocity [tcp_vel = dxe]
    smm_robot_kin_solver.SpatialJacobian_Tool_1(smm_robot_kin_solver.ptr2Jsp1);
    tcp_vel = smm_robot_kin_solver.updateSpatialVelocityTCP(q, dq);
    ROS_INFO("[idosc_simple_JointStateCallback] TCP Velocity: x: %f, y: %f, z: %f", tcp_vel.x(), tcp_vel.y(), tcp_vel.z());
    //smm_robot_viz_solver.publishTCPvel(tcp_pos,tcp_vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "updateCartesianState_idosc_simple");
    ros::NodeHandle nh;

    // Debug statement
    ROS_INFO("[updateCartesianState_idosc_simple] Initializing robot structure");

    // Initialize the robot structure
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    // Debug statement
    ROS_INFO("[updateCartesianState_idosc_simple] Initializing shared library");

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[updateCartesianState_idosc_simple] Failed to initialize shared library.");
        return -1;
    }

    // Initialize the shared library for robot analytical solvers using screws
    ROS_INFO("[updateCartesianState_idosc_simple] Initialized Shared Library.");
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();  
    ScrewsVisualization& smm_robot_viz_solver = my_shared_lib.get_screws_visualization_solver(); 

    // Subscriber to gazebo robot joint states
    ros::Subscriber joint_current_state_idosc_simple_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(idosc_simple_JointStateCallback, _1, boost::ref(smm_robot_kin_solver), boost::ref(smm_robot_viz_solver))
    );

    // Define publisher for custom IdoscCurrent message
    ros::Publisher idosc_cartesian_current_state_pub = nh.advertise<smm_control::IdoscCurrent>("/cartesian_current_state", 10);

    // Run in loop
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // Create and populate the IdoscCurrent message
        smm_control::IdoscCurrent idosc_msg;
        
        // Fill the pose part with tcp_pos
        idosc_msg.pose_cur.position.x = tcp_pos.x();
        idosc_msg.pose_cur.position.y = tcp_pos.y();
        idosc_msg.pose_cur.position.z = tcp_pos.z();
        // Set orientation to identity quaternion or compute separately if necessary
        idosc_msg.pose_cur.orientation.w = 1.0;
        idosc_msg.pose_cur.orientation.x = 0.0;
        idosc_msg.pose_cur.orientation.y = 0.0;
        idosc_msg.pose_cur.orientation.z = 0.0;

        // Fill the twist part with tcp_vel
        idosc_msg.twist_cur.linear.x = tcp_vel.x();
        idosc_msg.twist_cur.linear.y = tcp_vel.y();
        idosc_msg.twist_cur.linear.z = tcp_vel.z();
        // Set angular velocity to 0 or compute separately if necessary
        idosc_msg.twist_cur.angular.x = 0.0;
        idosc_msg.twist_cur.angular.y = 0.0;
        idosc_msg.twist_cur.angular.z = 0.0;

        // Publish the IdoscCurrent message
        idosc_cartesian_current_state_pub.publish(idosc_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}