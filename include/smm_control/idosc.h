#ifndef IDOSC_H
#define IDOSC_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include "smm_screws/robot_shared.h"
#include "smm_control/IdoscError.h"
#include "smm_control/IdoscCurrent.h"
#include "smm_control/IdoscDesired.h"

#define IDOSC_DOF 3
#define IDOSC_STATE_DIM 6
#define IDOSC_POS_DIM 3
#define IDOSC_VEL_DIM 3

namespace smm_control_ns {

class idosc : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    //idosc(); 
    //~idosc(); 

    //bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh, robot_shared* robot_ptr) override;
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
    bool smm_custom_init(robot_shared* robot_ptr);

    void starting(const ros::Time &time) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
    void stopping(const ros::Time &time) override;

private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void desiredStateCallback(const smm_control::IdoscDesired::ConstPtr& msg);
    void compute_u_vector();
    void publishIdoscError();
    void publishIdoscCurrent();
    bool initializePDgains();

    void updateKinDynJointStates();

    ros::NodeHandle nh_;  // Private NodeHandle

    // Handles to the joints
    std::vector<hardware_interface::JointHandle> joint_handles_;

    // ROS interfaces
    ros::Subscriber joint_state_sub_;           // reads q,dq from robot
    ros::Subscriber desired_tcp_state_sub_;     // reads xd,dxd from topic
    ros::Publisher idosc_current_pub_; 
    ros::Publisher idosc_error_pub_;

    // Kinematics and dynamics
    robot_shared* smm_robot_;
    ScrewsKinematics* kin_;
    ScrewsDynamics* dyn_;

    // Controller parameters
    Eigen::Vector3f q_, dq_;   // current joints pos-vel
    float qf_[IDOSC_DOF];
    float dqf_[IDOSC_DOF];
    Eigen::Vector3f xe_, dxe_; // current tcp pos-vel
    Eigen::Vector3f xd_, dxd_, ddxd_; // desired tcp pos-vel-accel
    Eigen::Matrix<float, IDOSC_STATE_DIM, 1> X_, Xhat_; // idosc state vectors
    Eigen::Matrix3f Kp_, Kd_; // gain matrices of idosc internal dynamics in y vector calculation
    size_t num_joints_;
    std::vector<std::string> joint_names_;
};

} // namespace smm_control

#endif // IDOSC_H