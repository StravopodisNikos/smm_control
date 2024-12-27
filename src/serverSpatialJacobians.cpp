#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/GetSpatialJacobians.h"  // Include the service header
#include <mutex>

Eigen::Matrix<float, 6, DOF>  sp1_jacobian_matrix;
Eigen::Matrix<float, 6, DOF>  sp2_jacobian_matrix;
//Eigen::Matrix<float, 6, 1>* ptr2jsp1_col[DOF];
//Eigen::Matrix<float, 6, 1>* ptr2jsp2_col[DOF];

std::mutex jacobian_mutex;  // Mutex for protecting Jacobian updates

/*
 *  [27-12-24] Calculates all spatial Jacobians. Don't change sequence of method calls!
 *             Implements rosservice.
 */

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state, ScrewsKinematics& smm_robot_kin_solver) {
    if (joint_state->position.size() < DOF || joint_state->velocity.size() < DOF) {
        ROS_WARN("[serverSpatialJacobians/JointStateCallback] Expected 3 joint positions and velocities, received %zu positions and %zu velocities", 
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

    // Lock the mutex to update Jacobians safely
    std::lock_guard<std::mutex> lock(jacobian_mutex);

    // Calculate the Spatial Jacobian 1
    smm_robot_kin_solver.ForwardKinematicsTCP();
    smm_robot_kin_solver.ForwardKinematics3DOF_1(); // updates g[]
    smm_robot_kin_solver.SpatialJacobian_Tool_1();  

    // here must assign to the local ptr and array!!!!
     // Local storage for Spatial Jacobian 1

    // Copy data from ptr2Jsp1 to sp1_jacobian_matrix
    for (size_t col = 0; col < DOF; col++) {
        sp1_jacobian_matrix.col(col) = *(smm_robot_kin_solver.ptr2Jsp1[col]);
    }

    // Log the Spatial Jacobian 1 for debugging
    ROS_INFO_STREAM("[serverSpatialJacobians/JointStateCallback] Spatial Jacobian 1:\n" << sp1_jacobian_matrix);
    
    // Calculate the Spatial Jacobian 2
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // updates g[]
    smm_robot_kin_solver.SpatialJacobian_Tool_2();

    // here must assign to the local ptr and array!!!!

    // Copy data from ptr2Jsp2 to sp2_jacobian_matrix
    for (size_t col = 0; col < DOF; col++) {
        sp2_jacobian_matrix.col(col) = *(smm_robot_kin_solver.ptr2Jsp2[col]);
    }

    // Log the Spatial Jacobian 2 for debugging
    ROS_INFO_STREAM("Spatial Jacobian 2:\n" << sp2_jacobian_matrix);
}

// Service callback to provide requested Jacobians
bool sendJacobians(smm_control::GetSpatialJacobians::Request &req, smm_control::GetSpatialJacobians::Response &res) {
    // Initialize response arrays to zero
    for (int i = 0; i < 18; i++) {
        res.sp1_jacobian[i] = 0.0;
        res.sp2_jacobian[i] = 0.0;
    }

    // If client requests spatial1 Jacobian
    if (req.get_sp1_jacobian) {
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                res.sp1_jacobian[i * 3 + j] = sp1_jacobian_matrix(i, j); // row-major order => i * 3 + j !!!
            }
        }
        //ROS_INFO("[serverSpatialJacobians/sendJacobians] Spatial Jacobian 1 sent.");
    }

    // If client requests spatial2 Jacobian
    if (req.get_sp2_jacobian) {
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                res.sp2_jacobian[i * 3 + j] = sp2_jacobian_matrix(i, j); // row-major order => i * 3 + j !!!
            }
        }
        //ROS_INFO("[serverSpatialJacobians/sendJacobians] Spatial Jacobian 2 sent.");
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serverSpatialJacobians");
    ros::NodeHandle nh;

    // Initialize the robot structure
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        //ROS_ERROR("[serverSpatialJacobians] Failed to initialize shared library.");
        return -1;
    }

    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();

    // Subscriber to gazebo robot joint states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(JointStateCallback, _1, boost::ref(smm_robot_kin_solver))
    );

    // Advertise the Jacobian service
    ros::ServiceServer service = nh.advertiseService("GetSpatialJacobians", sendJacobians);

    // Run in loop
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}