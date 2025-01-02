#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <smm_control/CustomTcpState.h>
#include <smm_control/GetJacobians.h>
#include <Eigen/Dense>
#include "smm_control/timing.h"

class IKNode {
public:
    IKNode() : nh_(), loop_rate_(IKP_LOOP_RATE), desired_position_received_(false) {
        // Subscriber to desired TCP state
        desired_tcp_state_sub_ = nh_.subscribe("/tcp_desired_state", 1, &IKNode::tcpDesStateCallback, this);     // get x_tcp_d
        // Subscriber to current TCP state
        current_tcp_state_sub_ = nh_.subscribe("/tcp_current_state", 1, &IKNode::tcpCurStateCallback, this);     // get x_tcp
        // Subscriber to current joint state
        current_joint_state_sub_ = nh_.subscribe("/joint_current_state", 1, &IKNode::jointCurStateCallback, this); // get q   
        // Publisher for desired joint positions
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_desired_state", 1); 

        ROS_INFO("IKNode initialized.");
    }

    void run() {
        while (ros::ok()) {
            ros::spinOnce();

            // Perform IK if a valid desired position is available
            if (desired_position_received_) {
                Eigen::Vector3f joint_positions = Eigen::Vector3f::Zero();
                Eigen::Vector3f initial_position = Eigen::Vector3f::Zero(); // Initial TCP position
                if (computeInverseKinematics(desired_position_, initial_position, J, joint_positions)) {
                    publishJointStates(joint_positions);
                }
            }

            loop_rate_.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber desired_tcp_state_sub_;
    ros::Subscriber current_tcp_state_sub_;
    ros::Subscriber current_joint_state_sub_;
    ros::Publisher joint_state_pub_;
    ros::Rate loop_rate_;

    Eigen::Matrix3f J;
    Eigen::Vector3f current_position_;
    Eigen::Vector3f desired_position_;
    bool desired_position_received_;

    // Callback for desired TCP state
    void tcpDesStateCallback(const smm_control::CustomTcpState::ConstPtr& msg) {
        desired_position_ << msg->position[0], msg->position[1], msg->position[2];
        desired_position_received_ = true;
        ROS_INFO_STREAM("Received desired position: [" << desired_position_.transpose() << "]");
    }



    // Perform inverse kinematics using iterative Newton-Raphson method
    bool computeInverseKinematics(const Eigen::Vector3f& x_tcp_d, const Eigen::Vector3f& x_tcp_init,
                                   Eigen::Matrix3f& J, Eigen::Vector3f& q_d, int max_iter = 50, float tolerance = 1e-4) {
        Eigen::Vector3f q_guess = q_d; // Start with the initial guess
        Eigen::Vector3f delta_q;       // Change in joint positions
        Eigen::Vector3f x_error;       // TCP position error

        for (int i = 0; i < max_iter; ++i) {
            // Compute forward kinematics (current TCP position)
            Eigen::Vector3f x_current = x_tcp_init + J * q_guess; // add my custom forward kinematics 

            // Compute error in TCP position
            x_error = x_tcp_d - x_current;

            // Check convergence
            if (x_error.norm() < tolerance) {
                q_d = q_guess;
                // print total time for convergence (add the appropriate commands in any other place)
                return true; // Converged successfully
            }

            // Compute the joint position update
            delta_q = J.completeOrthogonalDecomposition().pseudoInverse() * x_error; // we will get the inverse from the service
            float dt = 1.0 / loop_rate_.expectedCycleTime().toSec(); // also in the main use a loop rate specified by IKP_LOOP_RATE ct

            // Update guess
            q_guess += delta_q * dt ; // here use the dt
        }

        ROS_WARN("Inverse Kinematics did not converge within max iterations.");
        return false; // Failed to converge
    }

    // Publish joint states
    void publishJointStates(const Eigen::Vector3f& q_d) {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = {"joint1", "joint2", "joint3"};
        joint_state_msg.position = {q_d[0], q_d[1], q_d[2]};
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_pub_.publish(joint_state_msg);

        ROS_INFO_STREAM("Published joint positions: [" << q_d.transpose() << "]");
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "num_ikp1");
    IKNode ik_node;
    ik_node.run();
    return 0;
}