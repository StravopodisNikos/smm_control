#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <smm_screws/robot_shared.h>
#include <smm_control/idosc.h>

// this can be set to accept Structure3Pseudos as arg in the launch file
int main(int argc, char** argv) {
    ros::init(argc, argv, "initialize_robot_shared_node");
    ros::NodeHandle nh;

    // Initialize the robot_shared instance
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[initialize_robot_shared_node] Failed to initialize shared library.");
        return -1;
    }
    ROS_INFO("[initialize_robot_shared_node] Initialized Shared Library.");

    // Load the controller
    controller_manager::ControllerManager cm(&my_shared_lib, nh);

    // Assuming your controller manager spawns the idosc controller
    while (ros::ok()) {
        ros::spinOnce();
        cm.update(ros::Time::now(), ros::Duration(0.01));
    }

    return 0;
}