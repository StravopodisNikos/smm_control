#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/GetDynamics.h"  // Include the service header
#include "smm_control/timing.h"

Eigen::Matrix3f Mass_Matrix;       // Marc Marquez (!), or Mass Matrix
Eigen::Matrix3f Coriolis_Matrix;   // Coriolis Matrix
Eigen::Matrix<float, 3, 1> Gravity_Vector; // Gravity Vector

/*
 *  [30-9-24] Calculates all robot dynamic matrices, implementing screw theory tools!
 *            Implements rosservice.
 */

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state, ScrewsDynamics& smm_robot_dyn_solver) {
    if (joint_state->position.size() < DOF || joint_state->velocity.size() < DOF) {
        ROS_WARN("[serverRobotDynamics/JointStateCallback] Expected 3 joint positions and velocities, received %zu positions and %zu velocities", 
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
    smm_robot_dyn_solver.updateJointPos(q);
    smm_robot_dyn_solver.updateJointVel(dq);

    // Calculate Mass Matrix
    Mass_Matrix = smm_robot_dyn_solver.MassMatrix();

    // Calculate Coriolis Matrix
    Coriolis_Matrix = smm_robot_dyn_solver.CoriolisMatrix();

    // Calculate Gravity Vector
    Gravity_Vector = smm_robot_dyn_solver.GravityVectorAnalytical();
}

// Service callback to provide requested Dynamic matrices
bool sendDynamics(smm_control::GetDynamics::Request &req, smm_control::GetDynamics::Response &res) {
    // Initialize response arrays to zero
    for (int i = 0; i < 9; i++) {
        res.Mass[i] = 0.0;
        res.Coriolis[i] = 0.0;
    }
    for (int i = 0; i < 3; i++) {
        res.Gravity[i] = 0.0;
    }

    // If client requests Mass Matrix
    if (req.get_Mass) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.Mass[i * 3 + j] = Mass_Matrix(i, j);
            }
        }
        //ROS_INFO("[serverRobotDynamics/sendDynamics] Mass Matrix sent.");
    }

    // If client requests Coriolis Matrix
    if (req.get_Coriolis) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.Coriolis[i * 3 + j] = Coriolis_Matrix(i, j);
            }
        }
        //ROS_INFO("[serverRobotDynamics/sendDynamics] Coriolis Matrix sent.");
    }

    // If client requests Gravity Vector
    if (req.get_Gravity) {
        for (int i = 0; i < 3; i++) {
            res.Gravity[i] = -Gravity_Vector(i);
        }
        //ROS_INFO("[serverRobotDynamics/sendDynamics] Gravity vector sent.");
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serverRobotDynamics");
    ros::NodeHandle nh;

    // Initialize the robot structure
    int str_digit_loc;
    if (!nh.getParam("/str_digit", str_digit_loc)) {
        ROS_ERROR("[serverOperationalSpaceMatrices] Failed to get str_digit parameter.");
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
            ROS_ERROR("[serverOperationalSpaceMatrices] Invalid str_digit value. Must be 2, 3, or 4.");
            return -1;
    }

    // Debug statement
    ROS_INFO("[serverRobotDynamics] Initializing shared library");

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[serverRobotDynamics] Failed to initialize shared library.");
        return -1;
    }

    // Initialize the shared library for robot analytical solvers using screws
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver(); 

    // Subscriber to gazebo robot joint states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(JointStateCallback, _1, boost::ref(smm_robot_dyn_solver))
    );

    // Advertise the Jacobian service
    ros::ServiceServer service = nh.advertiseService("GetRobotDynamics", sendDynamics);

    // Run in loop
    ros::Rate loop_rate(UPDATE_SERVER_DYNAMICS_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}