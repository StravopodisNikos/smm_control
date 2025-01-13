#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <smm_control/IdoscDesired.h>
#include <geometry_msgs/Pose.h>

class TcpGoalVisualizer {
public:
    TcpGoalVisualizer() {
        // Initialize subscriber
        tcp_state_sub_ = nh_.subscribe("/ridosc_desired_state", 10, &TcpGoalVisualizer::tcpStateCallback, this);
        // Initialize publisher
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("tcp_goal_pos_marker", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber tcp_state_sub_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;

    void tcpStateCallback(const smm_control::IdoscDesired::ConstPtr& msg) {
        // Extract position from the message
        geometry_msgs::Point position;
        position.x = msg->pose_des.position.x;
        position.y = msg->pose_des.position.y;
        position.z = msg->pose_des.position.z;

        // Create and configure a marker
        marker_.header.frame_id = "world";  // Replace with your robot's base frame
        marker_.header.stamp = ros::Time::now();
        marker_.ns = "tcp_goal";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::SPHERE;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.pose.position = position;
        marker_.pose.orientation.w = 1.0;
        marker_.scale.x = 0.005;  // Diameter of the sphere
        marker_.scale.y = 0.005;
        marker_.scale.z = 0.005;
        marker_.color.r = 1.0;  // Red color
        marker_.color.g = 0.0;
        marker_.color.b = 0.0;
        marker_.color.a = 0.5;  // 50% opaque

        // Publish the marker
        marker_pub_.publish(marker_);
        /*ROS_INFO_STREAM("Published TCP Goal Marker at position: [" 
                        << position.x << ", " << position.y << ", " << position.z << "]");*/
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualizeTcpGoal");
    TcpGoalVisualizer visualizer;
    ros::spin();
    return 0;
}
