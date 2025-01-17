#include <ros/ros.h>
#include "smm_kdl_utils/smm_kin_tools.h" // Make sure this path is correct
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

void publishPoseMarker(const ros::Publisher& marker_pub, const geometry_msgs::Pose& pose, const std::string& frame_id, const std::string& ns, int id, const std::string& type, const Eigen::Vector3f& scale, const Eigen::Vector4f& color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;

    if (type == "arrow") {
        marker.type = visualization_msgs::Marker::ARROW;
    } else if (type == "sphere") {
        marker.type = visualization_msgs::Marker::SPHERE;
    } else if (type == "cube") {
        marker.type = visualization_msgs::Marker::CUBE;
    } else {
        ROS_WARN_STREAM("Invalid marker type: " << type << ". Using default (arrow).");
        marker.type = visualization_msgs::Marker::ARROW;
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_smm_kdl_ikp");
    ros::NodeHandle nh;

    // Initialize publishers *before* the loop
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("pose_marker", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("target_pose", 1);
    ros::Publisher intermediate_pose_pub = nh.advertise<geometry_msgs::Pose>("intermediate_pose", 1);

    std::string base_link_name_str;
    if (!nh.getParam("base_link_name", base_link_name_str)) {
        ROS_ERROR("Failed to get 'base_link_name' parameter.");
        return -1;
    }

    std::string tcp_link_name_str;
    if (!nh.getParam("tcp_link_name", tcp_link_name_str)) {
        ROS_ERROR("Failed to get 'tcp_link_name' parameter.");
        return -1;
    }

    SmmKinTools kin_tools(base_link_name_str, tcp_link_name_str, nh);

    Eigen::Matrix4d T_last_to_tcp;
    T_last_to_tcp << 1, 0, 0, 0.00,
                    0, 1, 0, 0.00,
                    0, 0, 1, 0.0615,
                    0, 0, 0, 1;

    std::vector<double> des_tcp_pos_vec;
    if (!nh.getParam("des_tcp_pos", des_tcp_pos_vec) || des_tcp_pos_vec.size() != 3) {
        ROS_ERROR("[test_smm_kdl_ikp] Missing or invalid parameter: des_tcp_pos");
        return -1; // Return -1 for error
    }

    ros::Rate loop_rate(10); 

    while (ros::ok()) { 
        Eigen::Vector3f x_tcp_d;
        x_tcp_d[0] = static_cast<float>(des_tcp_pos_vec[0]);
        x_tcp_d[1] = static_cast<float>(des_tcp_pos_vec[1]);
        x_tcp_d[2] = static_cast<float>(des_tcp_pos_vec[2]);

        geometry_msgs::Pose target_pose;
        target_pose.position.x = x_tcp_d[0];
        target_pose.position.y = x_tcp_d[1];
        target_pose.position.z = x_tcp_d[2];
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 1.0;

        pose_pub.publish(target_pose); // Publish the pose INSIDE the loop

        Eigen::Vector3f scale(0.025, 0.025, 0.025);
        Eigen::Vector4f color(1.0, 0.0, 0.0, 0.5);
        publishPoseMarker(marker_pub, target_pose, base_link_name_str, "target_pose_namespace", 0, "sphere", scale, color);

        std::vector<double> joint_positions;
        geometry_msgs::Pose massage_origin_frame_pose;
        bool success = kin_tools.computeIK(target_pose, massage_origin_frame_pose, joint_positions, T_last_to_tcp, nh);
        
        intermediate_pose_pub.publish(massage_origin_frame_pose);
        Eigen::Vector4f color2(0.0, 1.0, 0.0, 0.5);
        publishPoseMarker(marker_pub, massage_origin_frame_pose, base_link_name_str, "intermediate_pose_namespace", 0, "sphere", scale, color2);

        Eigen::Vector3f q_d = Eigen::Vector3f::Zero();
        if (success) {
            for (size_t i = 0; i < joint_positions.size(); ++i) {
                if (i < 3) {
                    q_d[i] = static_cast<float>(joint_positions[i]);
                }
            }
            std::cout << "[test_smm_kdl_ikp] Inverse Kinematics Solution: " << q_d.transpose() << std::endl;
        } else {
            ROS_WARN("[test_smm_kdl_ikp] IK computation failed.");
        }

        ros::spinOnce();        
        loop_rate.sleep();
    } // End of the publishing loop

    return 0;
}