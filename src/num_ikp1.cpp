#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <smm_control/CustomTcpState.h>
#include <smm_control/GetJacobians.h>
#include <Eigen/Dense>

class IKNode {
public:
    IKNode() : nh_(), loop_rate_(50), J(Eigen::Matrix3f::Zero()), desired_position_received_(false) {
        // Subscriber to desired TCP state
        desired_tcp_state_sub_ = nh_.subscribe("/tcp_desired_state", 1, &IKNode::tcpStateCallback, this);

        // Publisher for joint positions
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/ik_joint_states", 1);

        // Service client for Jacobian retrieval
        jacobian_client_ = nh_.serviceClient<smm_control::GetJacobians>("GetOperationalJacobians");

        ROS_INFO("IKNode initialized.");
    }

    void run() {
        while (ros::ok()) {
            ros::spinOnce();

            // Retrieve the Jacobian
            if (!getJacobianFromService()) {
                ROS_WARN_THROTTLE(5, "Unable to retrieve Jacobian. Retrying...");
                continue;
            }

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
    ros::Publisher joint_state_pub_;
    ros::ServiceClient jacobian_client_;
    ros::Rate loop_rate_;

    Eigen::Matrix3f J;  // Jacobian matrix
    Eigen::Vector3f desired_position_;
    bool desired_position_received_;

    // Callback for desired TCP state
    void tcpStateCallback(const smm_control::CustomTcpState::ConstPtr& msg) {
        desired_position_ << msg->position[0], msg->position[1], msg->position[2];
        desired_position_received_ = true;
        ROS_INFO_STREAM("Received desired position: [" << desired_position_.transpose() << "]");
    }

    // Retrieve Jacobian from service
    bool getJacobianFromService() {
        smm_control::GetJacobians srv;
        srv.request.get_op = true;

        if (!jacobian_client_.call(srv)) {
            ROS_ERROR("[IKNode] Failed to call Jacobian service.");
            return false;
        }

        // Convert flat array to 3x3 matrix
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                J(i, j) = srv.response.op_jacobian[i * 3 + j];
            }
        }

        ROS_INFO_STREAM("[IKNode] Retrieved Jacobian:\n" << J);
        return true;
    }

    // Perform inverse kinematics using iterative Newton-Raphson method
    bool computeInverseKinematics(const Eigen::Vector3f& x_tcp_d, const Eigen::Vector3f& x_tcp_init,
                                   Eigen::Matrix3f& J, Eigen::Vector3f& q_d, int max_iter = 50, float tolerance = 1e-4) {
        Eigen::Vector3f q_guess = q_d; // Start with the initial guess
        Eigen::Vector3f delta_q;       // Change in joint positions
        Eigen::Vector3f x_error;       // TCP position error

        for (int i = 0; i < max_iter; ++i) {
            // Compute forward kinematics (current TCP position)
            Eigen::Vector3f x_current = x_tcp_init + J * q_guess;

            // Compute error in TCP position
            x_error = x_tcp_d - x_current;

            // Check convergence
            if (x_error.norm() < tolerance) {
                q_d = q_guess;
                return true; // Converged successfully
            }

            // Compute the joint position update
            delta_q = J.completeOrthogonalDecomposition().pseudoInverse() * x_error;

            // Update guess
            q_guess += delta_q;
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