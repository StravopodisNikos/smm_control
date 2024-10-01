#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/GetJacobians.h"  // Include the service header
#include <mutex>

Eigen::Matrix3f op_jacobian_matrix = Eigen::Matrix3f::Zero();
Eigen::Matrix3f inverse_jacobian_matrix = Eigen::Matrix3f::Zero();
Eigen::Matrix3f derivative_jacobian_matrix = Eigen::Matrix3f::Zero();
std::mutex jacobian_mutex;  // Mutex for protecting Jacobian updates

/*
 *  [30-9-24] Calculates all task-space Jacobians. Don't change sequence of method calls!
 *            Implements rosservice. Here all operational jacobians are calculated
 */

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state, ScrewsKinematics& smm_robot_kin_solver) {
    if (joint_state->position.size() < DOF || joint_state->velocity.size() < DOF) {
        ROS_WARN("[serverToolJacobians/JointStateCallback] Expected 3 joint positions and velocities, received %zu positions and %zu velocities", 
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

    // Calculate the Operational Jacobian
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // updates g[]
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    smm_robot_kin_solver.OperationalSpaceJacobian(); // updates ptr2Jop
    Eigen::Matrix3f* op_jacobian_ptr = smm_robot_kin_solver.getOperationalJacobian();
    if (op_jacobian_ptr != nullptr) {
        op_jacobian_matrix = *op_jacobian_ptr;  // Correct assignment
    } else {
        ROS_WARN("[serverToolJacobians/JointStateCallback] Operational Jacobian pointer is null.");
    }

    // Calculate the inverse of Operational Jacobian
    smm_robot_kin_solver.inverseOperationalSpaceJacobian();
    Eigen::Matrix3f* inv_jacobian_ptr = smm_robot_kin_solver.getInverseOperationalJacobian();
    if (inv_jacobian_ptr != nullptr) {
        inverse_jacobian_matrix = *inv_jacobian_ptr;  // Correct assignment
    } else {
        ROS_WARN("[serverToolJacobians/JointStateCallback] Inverse Operational Jacobian pointer is null.");
    }

    // Calculate the First Time Derivative of Operational Jacobian
    smm_robot_kin_solver.DtBodyJacobian_Tool_1(); // updates ptr2dJbd_t_1, needs updated g[]
    smm_robot_kin_solver.SpatialJacobian_Tool_1(); // updates ptr2Jsp1
    smm_robot_kin_solver.ToolVelocityTwist(ScrewsKinematics::typ_jacobian::SPATIAL); // needs ptr2Jsp1, needed for dRst
    smm_robot_kin_solver.DtOperationalSpaceJacobian(); // updates ptr2dJop
    Eigen::Matrix3f* dt_jacobian_ptr = smm_robot_kin_solver.getDerivativeOperationalJacobian();
    if (dt_jacobian_ptr != nullptr) {
        derivative_jacobian_matrix = *dt_jacobian_ptr;  // Correct assignment
    } else {
        ROS_WARN("[serverToolJacobians/JointStateCallback] First Time Derivative of Operational Jacobian pointer is null.");
    }
}

// Service callback to provide requested Jacobians
bool sendJacobians(smm_control::GetJacobians::Request &req, smm_control::GetJacobians::Response &res) {
    // Initialize response arrays to zero
    for (int i = 0; i < 9; i++) {
        res.op_jacobian[i] = 0.0;
        res.inv_op_jacobian[i] = 0.0;
        res.dt_op_jacobian[i] = 0.0;
    }

    // If client requests operational Jacobian
    if (req.get_op_jacobian) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.op_jacobian[i * 3 + j] = op_jacobian_matrix(i, j);
            }
        }
        ROS_INFO("[serverToolJacobians/sendJacobians] Operational Jacobian sent.");
    }

    // If client requests inverse operational Jacobian
    if (req.get_inv_op_jacobian) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.inv_op_jacobian[i * 3 + j] = inverse_jacobian_matrix(i, j);
            }
        }
        ROS_INFO("[serverToolJacobians/sendJacobians] Inverse Operational Jacobian sent.");
    }

    // If client requests time derivative of operational Jacobian
    if (req.get_dt_op_jacobian) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.dt_op_jacobian[i * 3 + j] = derivative_jacobian_matrix(i, j);
            }
        }
        ROS_INFO("[serverToolJacobians/sendJacobians] Time Derivative of Operational Jacobian sent.");
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serverToolJacobians");
    ros::NodeHandle nh;

    // Initialize the robot structure
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[serverToolJacobians] Failed to initialize shared library.");
        return -1;
    }

    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();

    // Subscriber to gazebo robot joint states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(JointStateCallback, _1, boost::ref(smm_robot_kin_solver))
    );

    // Advertise the Jacobian service
    ros::ServiceServer service = nh.advertiseService("GetOperationalJacobians", sendJacobians);

    // Run in loop
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}