#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state, 
                                ScrewsKinematics& smm_robot_kin_solver) {
    if (joint_state->position.size() < DOF || joint_state->velocity.size() < DOF) {
        ROS_WARN("[updateToolJacobian/JointStateCallback] Expected 3 joint positions and velocities, received %zu positions and %zu velocities", 
                  joint_state->position.size(), joint_state->velocity.size());
        return;
    }

    float q[3] = {static_cast<float>(joint_state->position[0]),
                  static_cast<float>(joint_state->position[1]),
                  static_cast<float>(joint_state->position[2])};
    float dq[3] = {static_cast<float>(joint_state->velocity[0]),
                   static_cast<float>(joint_state->velocity[1]),
                   static_cast<float>(joint_state->velocity[2])};

    // Update joint states
    smm_robot_kin_solver.updateJointState(q, dq);

    // Calculate the Jacobian
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // updates g[]
    smm_robot_kin_solver.BodyJacobian_Tool_1(); // updates ptr2Jbd_t_1, needs updated g[]
    smm_robot_kin_solver.OperationalSpaceJacobian(); // updates ptr2Jop
    smm_robot_kin_solver.DtOperationalSpaceJacobian(); // updates ptr2dJop

    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "updateToolJacobian");
    ros::NodeHandle nh;

    // Initialize the robot structure
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[updateToolJacobian] Failed to initialize shared library.");
        return -1;
    }

    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();

    // Subscriber to gazebo robot joint states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(JointStateCallback, _1, boost::ref(smm_robot_kin_solver))
    );

    // Run in loop
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}