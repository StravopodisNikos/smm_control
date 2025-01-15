#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/GetImpedanceDynamics.h"  // Include the service header
#include "smm_control/timing.h"
#include <mutex>
#include <std_msgs/Float32.h>

Eigen::Matrix3f Mass_Matrix;            // Mass Matrix @ {q,dq}
Eigen::Matrix3f Coriolis_Matrix;        // Coriolis Matrix @ {q,dq}
Eigen::Matrix<float, 3, 1> Gravity_Vector; // Gravity Vector @ {q,dq}
Eigen::Matrix3f Lambda_Matrix;          // Mass Matrix @ TCP
Eigen::Matrix3f Gamma_Matrix;           // Coriolis Matrix @ TCP
Eigen::Vector3f Gamma_Vector = Eigen::Vector3f::Zero();
Eigen::Matrix<float, 3, 1> Fg_Vector;   // Gravity Vector @ TCP
Eigen::Matrix<float, 3, 1> dq_Vector;   // Joint velocities
Eigen::Vector3f bc = Eigen::Vector3f::Zero();
Eigen::Matrix3f op_jacobian_matrix = Eigen::Matrix3f::Zero();
Eigen::Matrix3f inverse_jacobian_matrix = Eigen::Matrix3f::Zero();
Eigen::Matrix3f derivative_jacobian_matrix = Eigen::Matrix3f::Zero();
std::mutex jacobian_mutex;  // Mutex for protecting Jacobian updates
float jacob_cond_number;

constexpr float BASE_LAMBDA_DLS = 0.005f;
constexpr float JACOB_COND_THRES = 10.0f;
/*
 *  [29-12-24] Calculates all robot dynamic matrices in operational space, implementing screw theory tools!
 */
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state,  ScrewsKinematics& smm_robot_kin_solver, ScrewsDynamics& smm_robot_dyn_solver, ros::Publisher& jacobian_cond_pub) {
    if (joint_state->position.size() < DOF || joint_state->velocity.size() < DOF) {
        ROS_WARN("[serverImpedanceDynamics/JointStateCallback] Expected 3 joint positions and velocities, received %zu positions and %zu velocities", 
                  joint_state->position.size(), joint_state->velocity.size());
        return;
    }

    float q[3] = {static_cast<float>(joint_state->position[0]),
                  static_cast<float>(joint_state->position[1]),
                  static_cast<float>(joint_state->position[2])};
    float dq[3] = {static_cast<float>(joint_state->velocity[0]),
                   static_cast<float>(joint_state->velocity[1]),
                   static_cast<float>(joint_state->velocity[2])};

    dq_Vector << dq[0], dq[1], dq[2];

    // I.1. Calculate Dynamics
    // I.2. Update joint states
    smm_robot_dyn_solver.updateJointPos(q);
    smm_robot_dyn_solver.updateJointVel(dq);

    // I.3. Calculate Mass Matrix
    Mass_Matrix = smm_robot_dyn_solver.MassMatrix();

    // I.4. Calculate Coriolis Matrix
    Coriolis_Matrix = smm_robot_dyn_solver.CoriolisMatrix(); // returns Coriolis @ C-space && updates Christoffelsymbols
    bc = smm_robot_dyn_solver.computeBetaCoriolis(smm_robot_dyn_solver.ChristoffelSymbols, dq_Vector); 

    // I.5. Calculate Gravity Vector
    Gravity_Vector = smm_robot_dyn_solver.GravityVectorAnalytical();

    // II. Calculate Operational Space Jacobian
    // II.1. Update joint states for kinematics solver
    smm_robot_kin_solver.updateJointState(q, dq);
    
    // II.2. Lock the mutex to update Jacobians safely
    std::lock_guard<std::mutex> lock(jacobian_mutex);

    // II.3. Calculate the Operational Jacobian
    smm_robot_kin_solver.ForwardKinematics3DOF_2(); // updates g[]
    smm_robot_kin_solver.BodyJacobian_Tool_1();
    std::unique_ptr<Eigen::Matrix3f> op_jacobian;
    try {
        op_jacobian = smm_robot_kin_solver.OperationalSpaceJacobian_ptr();
        // Access the matrix
        op_jacobian_matrix = *op_jacobian;
    } catch (const std::exception& e) {
        ROS_ERROR("[serverImpedanceDynamics/JointStateCallback] Failed to compute Operational Jacobian: %s", e.what());
    }

    // II.4. Evaluate the Condition Number of the Operational Jacobian
    jacob_cond_number = smm_robot_kin_solver.JacobianConditionNumber(op_jacobian_matrix);
    
    // Publish the Condition Number to topic /jacobian_condition_number
    std_msgs::Float32 cond_msg;
    cond_msg.data = jacob_cond_number;
    jacobian_cond_pub.publish(cond_msg);

    // II.5. Calculate the inverse Operational Jacobian
    std::unique_ptr<Eigen::Matrix3f> inv_jacobian;
    try {
        if (jacob_cond_number < JACOB_COND_THRES) {
            // Simple inverse
            //ROS_INFO("[serverImpedanceDynamics/JointStateCallback] Condition number is below threshold. Using simple inverse.");
            inv_jacobian = smm_robot_kin_solver.inverseOperationalSpaceJacobian_ptr();
        } else {
            // DLS inverse
            ROS_WARN("[serverImpedanceDynamics/JointStateCallback] Condition number exceeds threshold. Using Damped Least Squares (DLS) inverse.");
            inv_jacobian = smm_robot_kin_solver.DLSInverseJacobian(op_jacobian_matrix, BASE_LAMBDA_DLS);
        }

        if (inv_jacobian) {
            inverse_jacobian_matrix = *inv_jacobian; // Assign to local for further use
        } else {
            ROS_ERROR("[serverImpedanceDynamics/JointStateCallback] Failed to compute Jacobian inverse. Pointer is null.");
        }
    } catch (const std::runtime_error& e) {
        ROS_ERROR("[serverImpedanceDynamics/JointStateCallback] Exception while computing Jacobian inverse: %s", e.what());
    }

    // II.6 Calculate the First Time Derivative of Operational jacobian
    smm_robot_kin_solver.DtBodyJacobian_Tool_1(); // updates ptr2dJbd_t_1, needs updated g[]
    smm_robot_kin_solver.SpatialJacobian_Tool_1(); // updates ptr2Jsp1
    smm_robot_kin_solver.ToolVelocityTwist(ScrewsKinematics::typ_jacobian::SPATIAL); // needs ptr2Jsp1, needed for dRst
    std::unique_ptr<Eigen::Matrix3f> dt_jacobian;
    try {
        dt_jacobian = smm_robot_kin_solver.DtOperationalSpaceJacobian_ptr();
        // Access the matrix
        derivative_jacobian_matrix = *dt_jacobian;
    } catch (const std::exception& e) {
        ROS_ERROR("[serverImpedanceDynamics/JointStateCallback] Failed to compute Derivative Operational Jacobian: %s", e.what());
    }

    // III. Calculate Operational Space Dynamic matrices
    // III.1. Mass and Gravity matrices @ TCP
    Lambda_Matrix = inverse_jacobian_matrix.transpose() * Mass_Matrix * inverse_jacobian_matrix;
    Fg_Vector = inverse_jacobian_matrix.transpose() * (Gravity_Vector); // [2-1-25] Never put a "minus" sign in G. Never. Never.
    // III.2. Coriolis matrix @ TCP
    //Gamma_Matrix = inverse_jacobian_matrix.transpose() * Coriolis_Matrix * inverse_jacobian_matrix -  Lambda_Matrix * derivative_jacobian_matrix * inverse_jacobian_matrix; // De Luca
    Gamma_Vector = inverse_jacobian_matrix.transpose() * bc - Lambda_Matrix * derivative_jacobian_matrix * dq_Vector;
    return;
}

// Service callback to provide requested Dynamic matrices
bool sendDynamics(smm_control::GetImpedanceDynamics::Request &req, smm_control::GetImpedanceDynamics::Response &res) {
    // Initialize response arrays to zero
    for (int i = 0; i < 9; i++) {
        res.Lambda_imp[i] = 0.0;
        res.op_jacobian[i] = 0.0;
        //res.Gamma_imp[i] = 0.0;
    }
    for (int i = 0; i < 3; i++) {
        res.Gamma_imp[i] = 0.0;
        res.Fg_imp[i] = 0.0;
    }

    // If client requests Mass Matrix
    if (req.get_Lambda_imp) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.Lambda_imp[i * 3 + j] = Lambda_Matrix(i, j);
            }
        }
        //ROS_INFO("[serverImpedanceDynamics/sendDynamics] Mass Matrix sent.");
    }

    // If client requests Operational Jacobian
    if (req.get_op_jacobian) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.op_jacobian[i * 3 + j] = op_jacobian_matrix(i, j);
            }
        }
        //ROS_INFO("[serverImpedanceDynamics/sendDynamics] Operational Jacobian matrix sent.");
    }

    // If client requests Coriolis Vector
    if (req.get_Gamma_imp) {
        for (int i = 0; i < 3; i++) {
            //for (int j = 0; j < 3; j++) {
                res.Gamma_imp[i] = Gamma_Vector(i); // for double for + matrix: i * 3 + j
            //}
        }
        //ROS_INFO("[serverImpedanceDynamics/sendDynamics] Gamma vector sent.");
    }

    // If client requests Gravity Vector
    if (req.get_Fg_imp) {
        for (int i = 0; i < 3; i++) {
            res.Fg_imp[i] = Fg_Vector(i);
        }
        //ROS_INFO("[serverImpedanceDynamics/sendDynamics] Gravity vector sent.");
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serverImpedanceDynamics");
    ros::NodeHandle nh;

    // Initialize the robot structure
    int str_digit_loc;
    if (!nh.getParam("/str_digit", str_digit_loc)) {
        ROS_ERROR("[serverImpedanceDynamics] Failed to get str_digit parameter.");
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
            ROS_ERROR("[serverImpedanceDynamics] Invalid str_digit value. Must be 2, 3, or 4.");
            return -1;
    }

    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[serverImpedanceDynamics] Failed to initialize shared library.");
        return -1;
    }

    // Initialize smm_screws solvers     
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver(); 
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    
    // Publisher for Jacobian condition number
    ros::Publisher jacobian_cond_pub = nh.advertise<std_msgs::Float32>("/jacobian_condition_number", 10);

    // Get joint data from topic: joint_current_state || No: /smm_ros_gazebo/joint_states
    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_current_state", 10, 
        boost::bind(JointStateCallback, _1, boost::ref(smm_robot_kin_solver), boost::ref(smm_robot_dyn_solver), boost::ref(jacobian_cond_pub))
    );

    // Advertise the Operational Space Dynamics service
    ros::ServiceServer service = nh.advertiseService("GetImpedanceDynamics", sendDynamics);

    // Run in loop
    ros::Rate loop_rate(UPDATE_SERVER_DYNAMICS_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}