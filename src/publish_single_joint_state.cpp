#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_single_joint_state");
    ros::NodeHandle nh;

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time(0); // only for visualization
        joint_state.name = {"base_plate__base_link", "active_module_a_1__active_module_b_1", "active_module_a_2__active_module_b_2"};
        joint_state.position = {0.7, -0.1, 0.7};

        joint_pub.publish(joint_state);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
