#include <ros/ros.h>
#include <smm_screws/robot_shared.h>
#include <controller_manager/controller_manager.h>
#include <smm_control/idosc.h>

int main(int argc, char **argv)
{
    const std::string node_name = "example_idosc";
    ros::init(argc, argv, node_name.c_str());
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;

    // Initialize the robot structure
    ROS_INFO("[%s] Initializing robot structure", node_name.c_str());
    RobotAbstractBase* robot_ptr = new Structure3Pseudos();

    // Debug statement
    ROS_INFO("[%s] Initializing shared library", node_name.c_str());

    // Create an instance of your shared library with NodeHandle
    robot_shared my_shared_lib(robot_ptr, nh);
    if (!my_shared_lib.initializeSharedLib()) {
        ROS_ERROR("[%s] Failed to initialize shared library", node_name.c_str());
        return -1;
    }

    // Initialize the shared library for robot analytical solvers using screws
    ROS_INFO("[%s] Initialized Shared Library", node_name.c_str());
    ScrewsKinematics& smm_robot_kin_solver = my_shared_lib.get_screws_kinematics_solver();
    ScrewsDynamics& smm_robot_dyn_solver = my_shared_lib.get_screws_dynamics_solver(); 

    return 0;
}
