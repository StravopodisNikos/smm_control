#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/IdoscCurrent.h" // uses the same with idosc controller  
#include "smm_control/timing.h"

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

    // Extract and publish the TCP spatial position [tcp_pos = xe]
    tcp_pos = smm_robot_kin_solver.updatePositionTCP(q);

    // Extract and publish the Active Twists
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // Always run before use of SpatialJacobian_Tool_2() OR publishTwists().

    // Extract and publish the TCP spatial velocity [tcp_vel = dxe]
    smm_robot_kin_solver.SpatialJacobian_Tool_1(smm_robot_kin_solver.ptr2Jsp1);
    tcp_vel = smm_robot_kin_solver.updateSpatialVelocityTCP(q, dq);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "updateCartesianState_ridosc_simple");
    ros::NodeHandle nh;

    // Initialize the robot structure
    int str_digit_loc;
    if (!nh.getParam("/str_digit", str_digit_loc)) {
        ROS_ERROR("[updateCartesianState_ridosc_simple] Failed to get str_digit parameter.");
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
            ROS_ERROR("[updateCartesianState_ridosc_simple] Invalid str_digit value. Must be 2, 3, or 4.");
            return -1;
    }

    // Debug statement
    ROS_INFO("[updateCartesianState_ridosc_simple] Initializing shared library");

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[updateCartesianState_ridosc_simple] Failed to initialize shared library.");
        return -1;
    }

    // Initialize the shared library for robot analytical solvers using screws
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver();  
    ScrewsVisualization& smm_robot_viz_solver = my_shared_lib.get_screws_visualization_solver(); 

    // Subscriber to gazebo robot joint states
    ros::Subscriber joint_current_state_idosc_simple_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(idosc_simple_JointStateCallback, _1, boost::ref(smm_robot_kin_solver), boost::ref(smm_robot_viz_solver))
    );

    // Define publisher for custom IdoscCurrent message
    ros::Publisher idosc_cartesian_current_state_pub = nh.advertise<smm_control::IdoscCurrent>("/tcp_current_state", 10);

    // Run in loop
    ros::Rate loop_rate(UPDATE_CARTESIAN_CURRENT_STATE_RATE);
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
