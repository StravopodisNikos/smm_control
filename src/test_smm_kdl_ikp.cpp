#include <ros/ros.h>
#include "smm_kdl_utils/smm_kin_tools.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_smm_kdl_ikp");
    ros::NodeHandle nh;

    SmmKinTools kin_tools("base_plate", "massage_tool");

    Eigen::Vector3f x_tcp_d(0.3282, -0.3508, 0.3248); // Desired TCP position

    geometry_msgs::Pose target_pose;
    // Set desired pose x_tcp_d
    target_pose.position.x = x_tcp_d[0];
    target_pose.position.y = x_tcp_d[1];
    target_pose.position.z = x_tcp_d[2];
    // If orientation is needed, set it here
    target_pose.orientation.x = -1.0;
    target_pose.orientation.y = 0.5;
    target_pose.orientation.z = -1.0;
    target_pose.orientation.w = 1.0;  

    std::vector<double> joint_positions;
    bool success = kin_tools.computeIK(target_pose, joint_positions);

    Eigen::Vector3f q_d  = Eigen::Vector3f::Zero();
    if (success) {
        // Populate q_d with the solution and print
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            if (i < 3) { // Ensure bounds for Eigen::Vector3f
                q_d[i] = static_cast<float>(joint_positions[i]);
            }
        }
        // Print the joint positions
        std::cout << "Inverse Kinematics Solution: " << q_d.transpose() << std::endl;
    } else {
        ROS_WARN("IK computation failed.");
    }

    ros::spin();
    return 0;
}
