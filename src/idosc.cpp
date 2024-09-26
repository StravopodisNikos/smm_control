#include "smm_control/idosc.h"

namespace smm_control_ns
{
    idosc::idosc() : kin_(nullptr), dyn_(nullptr), smm_robot_(nullptr) {
    }

    idosc::~idosc() {
        // Clean up if needed
    }

    //bool idosc::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh, robot_shared* robot_ptr)
    //bool idosc::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
bool idosc::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh, robot_shared* robot_ptr)
    {
        ROS_INFO("[idosc/init] Starting initialization of idosc...");

        // 1. Start with nodehandle and shared class pointer
        nh_ = nh; // nh_("smm_ros_gazebo/idosc") ???
        smm_robot_ = robot_ptr;

        // 2. Initialize pointers to kinematics and dynamics libraries solvers
        kin_ = &smm_robot_->get_screws_kinematics_solver();
        dyn_ = &smm_robot_->get_screws_dynamics_solver();

        // 3. Load joint names from the parameter server
        if (!nh_.getParam("/smm_ros_gazebo/idosc/joints", joint_names_)) {
            ROS_ERROR("[idosc/init] No 'joints' parameter in namespace: %s", nh_.getNamespace().c_str());
            return false;
        } else {
            ROS_INFO("[idosc/init] Joint names loaded from ROS PARAMETER SERVER");
        }
        num_joints_ = joint_names_.size();
        ROS_INFO("[idosc/init] SMM TOTAL JOINTS : %zu", num_joints_ );

        // 4. Assign joint to hardware
        for (const auto &joint_name : joint_names_) {
            try {
                joint_handles_.push_back(hw->getHandle(joint_name));
            } catch (const hardware_interface::HardwareInterfaceException &e) {
                ROS_ERROR_STREAM("[idosc/init] Exception getting joint handle: " << e.what());
                return false;
            }
        }

        // 5. Print initial robot joints' positions/velocities
        for (size_t i = 0; i < num_joints_; ++i) {
            q_[i] = joint_handles_[i].getPosition();
            dq_[i] = joint_handles_[i].getVelocity();
            ROS_INFO("[idosc/init] Joint [%zu] Pos: [%f]  Vel: [%f]", num_joints_, q_[i], dq_[i] );
        }

        // Assign current position to 1st command (retain position in initialization)
        //command1_ = q_[0];
        //command2_ = q_[1];
        //command3_ = q_[2];

        // Assign the gain matrices
        if (!initializePDgains()) {
            ROS_ERROR("[idosc/init] Initialization of IDOSC PD gains failed");
            return false;
        }

        // Initialize command subscribers
        //sub_command1_ = nh_.subscribe<std_msgs::Float64>("command1", 1, &idosc::setCommandCB, this);

        // Initialize subscribers and publisher
        //joint_state_sub_ = nh_.subscribe("/joint_states", 10, &idosc::jointStateCallback, this);
        desired_tcp_state_sub_ = nh_.subscribe("/desired_tcp_state", 10, &idosc::desiredStateCallback, this);
        
        idosc_error_pub_ = nh_.advertise<smm_control::IdoscError>("/idosc_error_state", 10);
        idosc_current_pub_ = nh_.advertise<smm_control::IdoscCurrent>("/idosc_current_state", 10);

        // Initialize controller state vectors
        xd_.setZero(IDOSC_POS_DIM); dxd_.setZero(IDOSC_VEL_DIM); ddxd_.setZero(IDOSC_VEL_DIM);
        q_.setZero(IDOSC_POS_DIM); dq_.setZero(IDOSC_VEL_DIM);
        xe_.setZero(IDOSC_POS_DIM); dxe_.setZero(IDOSC_VEL_DIM);
        Xd_.setZero(); Xe_.setZero(); Xhat_.setZero();

        return true;
    }

    //bool idosc::smm_custom_init(robot_shared* robot_ptr) {
        // Initialize the robot pointer
    //    smm_robot_ = robot_ptr;

        // Initialize kinematics and dynamics libraries
    //    if (!smm_robot_->initializeSharedLib()) {
    //        ROS_ERROR("[idosc/smm_custom_init] Failed to initialize shared library.");
    //        return false;
    //    }
    //    kin_ = &smm_robot_->get_screws_kinematics_solver();
    //    dyn_ = &smm_robot_->get_screws_dynamics_solver();


    //    return true;
    //}

    //bool idosc::smm_custom_init() {
        // Initialize the robot pointer
    //    RobotAbstractBase* robot_ptr = new Structure3Pseudos();
    //    robot_shared my_shared_lib(robot_ptr, nh_);
        
    //    smm_robot_ = &my_shared_lib;

        // Initialize kinematics and dynamics libraries
    //    if (!my_shared_lib.initializeSharedLib()) {
    //        ROS_ERROR("[idosc/smm_custom_init] Failed to initialize shared library.");
    //        return false;
    //    }
    //    ROS_ERROR("[idosc/smm_custom_init] Initialized Shared Library.");
    //    kin_ = smm_robot_.get_screws_kinematics_solver();
    //    dyn_ = my_shared_lib.get_screws_dynamics_solver();

    //    return true;
    //}
    /*
    void idosc::setCommandCB(const std_msgs::Float64ConstPtr& msg)
    {
        command1_ = msg-> data;
    }
    */
    void idosc::starting(const ros::Time &time)
    {
        // Initialize the controller (e.g., reset error integrators, etc.)
    }



    void idosc::stopping(const ros::Time &time)
    {
        // Perform clean-up actions if necessary
    }

/*
    void idosc::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        for (size_t i = 0; i < num_joints_; ++i)
        {
            q_[i] = msg->position[i];
            dq_[i] = msg->velocity[i];

            qf_[i] = static_cast<float>(q_[i]);
            dqf_[i] = static_cast<float>(dq_[i]);
        }
        for (int i = 0; i < IDOSC_DOF; i++) {
            ROS_INFO("[idosc/jointStateCallback] q[ %d ]: %f", i+1, qf_[i]);
        }
        for (int i = 0; i < IDOSC_DOF; i++) {
            ROS_INFO("[idosc/jointStateCallback] dq[ %d ]: %f", i+1, dqf_[i]);
        }        
    }
*/

    void idosc::desiredStateCallback(const smm_control::IdoscDesired::ConstPtr& msg)
    {
        xd_[0] = msg->pose_des.position.x;
        xd_[1] = msg->pose_des.position.y;
        xd_[2] = msg->pose_des.position.z;
        dxd_[0] = msg->twist_des.linear.x;
        dxd_[1] = msg->twist_des.linear.y;
        dxd_[2] = msg->twist_des.linear.z;
    }

    void idosc::publishIdoscError()
    {
        smm_control::IdoscError error_msg;
        error_msg.twist_error.linear.x = dxd_[0] - dxe_[0];
        error_msg.twist_error.linear.y = dxd_[1] - dxe_[1];
        error_msg.twist_error.linear.z = dxd_[2] - dxe_[2];

        error_msg.pose_error.position.x = xd_[0] - xe_[0];
        error_msg.pose_error.position.y = xd_[1] - xe_[1];
        error_msg.pose_error.position.z = xd_[2] - xe_[2];

        idosc_error_pub_.publish(error_msg);
    }

    void idosc::publishIdoscCurrent()
    {
        smm_control::IdoscCurrent current_msg;

        current_msg.twist_cur.linear.x = dxe_[0];
        current_msg.twist_cur.linear.y = dxe_[1];
        current_msg.twist_cur.linear.z = dxe_[2];

        current_msg.pose_cur.position.x = xe_[0];
        current_msg.pose_cur.position.y = xe_[1];
        current_msg.pose_cur.position.z = xe_[2];

        idosc_current_pub_.publish(current_msg);
    }

    bool idosc::initializePDgains() {
        double kp, kd;
        if (!nh_.getParam("/smm_ros_gazebo/idosc/pd/p", kp) || !nh_.getParam("/smm_ros_gazebo/idosc/pd/d", kd)) {
            ROS_ERROR("[idosc/initializePDgains] Failed to get PD gains.");
            return false;
        }
        Kp_.setIdentity();
        Kd_.setIdentity();
        Kp_ *= kp;
        Kd_ *= kd;
        return true;
    }

    // ESSENTIALS FOR UPDATE FUNCTION
    // ALL MAGIC IS HERE-ALL CLASSES MUST WORK HERE!

    // 1. update function: executes the out-control loop. main controller function
    void idosc::update(const ros::Time &time, const ros::Duration &period)
    {
        // Read joint states from gazebo robot
        Eigen::Vector3f q, dq;
        for (size_t i = 0; i < num_joints_; ++i)
        {
            q[i] = joint_handles_[i].getPosition();
            dq[i] = joint_handles_[i].getVelocity();

            qf_[i] = static_cast<float>(q_[i]);
            dqf_[i] = static_cast<float>(dq_[i]);            
        }

        // Update kinematics and dynamics from smm_screws analytical tools
        updateKinDynJointStates();

        // Compute the IDOSC
        compute_u_vector();

        // Publish torques to joint controllers
        
        // Publish the error/current states
        //publishIdoscError();
        //publishIdoscCurrent();
    }

    // 2. updateKinDynJointStates: inserts the {q,dq} to the custom 
    //    kinematic and dynamic classes for smm analytical calculations
    void idosc::updateKinDynJointStates() 
    {
        kin_->updateJointState(qf_, dqf_);
        //dyn_->updateJointPos(qf_);
        //dyn_->updateJointVel(dqf_);
        return;
    }

    // 3. updateErrorState. New desired and current states
    //    must be previously updated
    void idosc::updateErrorState() 
    {
        // Xe_ : updated from updateCurrentState
        // Xd_ : updated from updateDesiredState
        Xhat_  = Xd_ - Xe_;

        //for (int i = 0; i < IDOSC_STATE_DIM; i++) {
        //    ROS_INFO("[idosc/updateErrorState] Xhat [ %d ]: %f", i, Xhat_(i));
        //}

        return;
    }

    // 3. updateErrorState. New desired and current states
    //    must be previously updated
    void idosc::updateCurrentState() 
    {
        // xe_ : updated from ScrewsKinematics::updatePositionTCP
        // dxe_: updated from ScrewsKinematics::updateSpatialVelocityTCP
        //xe_ = kin_->updatePositionTCP(qf_);

        //kin_->ForwardKinematics3DOF_2();
        //kin_->SpatialJacobian_Tool_1(kin_->ptr2Jsp1);
        //dxe_ = kin_->updateSpatialVelocityTCP(qf_, dqf_);

        Xe_.block<IDOSC_DOF,1>(0,0) = dxe_;
        Xe_.block<IDOSC_DOF,1>(IDOSC_DOF,0) = xe_;
        return;
    }

    void idosc::updateDesiredState() 
    {
        // xd_ : updated from idosc::desiredStateCallback
        // dxd_: updated from idosc::desiredStateCallback
        Xd_.block<IDOSC_DOF,1>(0,0) = dxd_;
        Xd_.block<IDOSC_DOF,1>(IDOSC_DOF,0) = xd_;
        return;
    }

    // 
    void idosc::compute_u_vector()
    {
        updateKinDynJointStates(); // update {q,dq}

        updateCurrentState();
        updateDesiredState();
        updateErrorState();


    }




}

PLUGINLIB_EXPORT_CLASS(smm_control_ns::idosc, controller_interface::ControllerBase) // allows dynamic loading of custom controller
