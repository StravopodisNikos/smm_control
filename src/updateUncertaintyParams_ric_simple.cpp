#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <smm_control/IdoscError.h>
#include <smm_control/IdoscCurrent.h>
#include <smm_control/RidoscAdaptiveParams.h> 
#include <smm_control/GetUncertaintyImpedanceDynamics.h> 
#include <smm_control/GetImpedanceDynamics.h> 
#include <smm_control/CustomTcpState.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "smm_control/timing.h"

// Define matrices and vectors 
Eigen::Matrix3f _Lambda;                         
Eigen::Matrix3f _Gamma;
Eigen::Vector3f _GammaVector;
Eigen::Vector3f _Fg;   
Eigen::Vector3f _n;                           
Eigen::Matrix3f _LambdaUn;                         
Eigen::Matrix3f _GammaUn;
Eigen::Vector3f _GammaVectorUn;
Eigen::Vector3f _FgUn;
Eigen::Vector3f _nUn;           
Eigen::Vector3f _ddx_d = Eigen::Vector3f::Zero();
Eigen::Matrix3f _DtLambda = Eigen::Matrix3f::Zero();
float _normInvLambda = 0.0f;
// Initialization only
Eigen::Vector3f qi_adapt_vec = Eigen::Vector3f::Zero();
Eigen::Vector3f pi_adapt_vec = Eigen::Vector3f::Zero();
Eigen::Vector3f ki_vec = Eigen::Vector3f::Zero();
Eigen::Vector3f lambda0_vec = Eigen::Vector3f::Zero();
Eigen::Matrix3f _lambda0Diag = Eigen::Matrix3f::Zero();
boost::array<double, 3> velocity_error = {0.0, 0.0, 0.0};
Eigen::Vector3f abs_velocity_error = Eigen::Vector3f::Zero();
Eigen::Vector3f _dx = Eigen::Vector3f::Zero();    
// Coeffs
Eigen::Matrix3f M_ij_bar;                         
Eigen::Matrix3f m_ij_bar;                         
Eigen::Vector3f c_i_bar;                         
Eigen::Vector3f n_i_bar;                         


// Global publisher for multiple fn access
ros::Publisher ridosc_adaptive_params_pub;

std::mutex data_mutex;

// Fixed params loaded from yaml ridosc/adaptive_params.yaml
struct Parameters {
    std::vector<float> lambda_0; // lambda_0: [17.5, 17.50, 17.5]
    float w_margin;              // w_margin: 1.5
    std::vector<float> ki;       // ki: [1.0, 1.0, 1.0]
    float pi_limit;              // pi_limit: 10.0
    float qi_limit;              // qi_limit: 10.0
} params;

// 1. Callback for /ridosc_error_state
void errorStateCallback(const smm_control::IdoscError::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Received a null IdoscError message.");
        return;
    }
    // Extract position error from the pose_error field
    velocity_error[0] = msg->twist_error.linear.x;
    velocity_error[1] = msg->twist_error.linear.y;
    velocity_error[2] = msg->twist_error.linear.z;

    abs_velocity_error[0] = static_cast<float>(std::abs(velocity_error[0]));
    abs_velocity_error[1] = static_cast<float>(std::abs(velocity_error[1]));
    abs_velocity_error[2] = static_cast<float>(std::abs(velocity_error[2]));
}

void currentTcpStateCallback(const smm_control::IdoscCurrent::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("Received a null IdoscCurrent message.");
        return;
    }
    _dx << msg->twist_cur.linear.x,
           msg->twist_cur.linear.y,
           msg->twist_cur.linear.z;
}

// 2. Function to call the service and retrieve current Operational Dynamic matrices
bool getDynamicsFromService(ros::NodeHandle& nh, bool get_JacobianMatrix, bool get_LambdaMatrix, bool get_GammaMatrix, bool get_FgVector) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetImpedanceDynamics>("GetImpedanceDynamics");
    smm_control::GetImpedanceDynamics srv;
 
    // Set flags in the request
    srv.request.get_op_jacobian = get_JacobianMatrix;
    srv.request.get_Lambda_imp = get_LambdaMatrix;
    srv.request.get_Gamma_imp = get_GammaMatrix;
    srv.request.get_Fg_imp = get_FgVector;

    if (client.call(srv)) {
        if (get_LambdaMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Lambda(i, j) = srv.response.Lambda_imp[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateUnParams_ric_simple/getDynamicsFromService] Lambda Matrix:\n" << _Lambda);
        }
        if (get_GammaMatrix) {
            for (int i = 0; i < 3; i++) {
                //for (int j = 0; j < 3; j++) {
                    //_Gamma(i, j) = srv.response.Gamma_imp[i * 3 + j];
                    _GammaVector(i) = srv.response.Gamma_imp[i];
                //}
            }
           // ROS_INFO_STREAM("[updateUnParams_ric_simple/getDynamicsFromService] TCP Gamma Vector:\n" << _GammaVector);
        }   
        if (get_FgVector) {
            for (int i = 0; i < 3; i++) {
                _Fg(i) = srv.response.Fg_imp[i];
            }
            //ROS_INFO_STREAM("[updateUnParams_ric_simple/getDynamicsFromService] TCP Gravity Vector:\n" << _Fg);
        }
        //_n = _Gamma * _dx + _Fg;
        _n = _GammaVector + _Fg;
        return true;        
    } else {
        ROS_ERROR("[updateUnParams_ric_simple/getDynamicsFromService] Failed to call service serverOperationalSpaceDynamics.");
        return false;
    }
}

// 3. Function to call the service and retrieve Modified Operational Dynamic matrices based on Uncertainty 
bool getUncertaintyDynamicsFromService(ros::NodeHandle& nh, bool get_LambdaUnMatrix, bool get_GammaUnmatrix, bool get_FgUnVector) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetUncertaintyImpedanceDynamics>("GetUncertaintyImpedanceDynamics");
    smm_control::GetUncertaintyImpedanceDynamics srv;
 
    // Set flags in the request
    srv.request.get_Lambda_un = get_LambdaUnMatrix;
    srv.request.get_Gamma_un = get_GammaUnmatrix;
    srv.request.get_Fg_un = get_FgUnVector;

    if (client.call(srv)) {
        if (get_LambdaUnMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _LambdaUn(i, j) = srv.response.Lambda_un[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateUnParams_ric_simple/getUncertaintyDynamicsFromService] Lambda Matrix:\n" << _LambdaUn);
        }
        if (get_GammaUnmatrix) {
            for (int i = 0; i < 3; i++) {
                //for (int j = 0; j < 3; j++) {
                    //_GammaUn(i,j) = srv.response.Gamma_un[i * 3 + j];
                    _GammaVectorUn(i) = srv.response.Gamma_un[i];
                //}
            }
           // ROS_INFO_STREAM("[updateUnParams_ric_simple/getUncertaintyDynamicsFromService] TCP Gamma Matrix:\n" << _GammaUn);
        }   
        if (get_FgUnVector) {
            for (int i = 0; i < 3; i++) {
                _FgUn(i) = srv.response.Fg_un[i];
            }
            //ROS_INFO_STREAM("[updateUnParams_ric_simple/getUncertaintyDynamicsFromService] TCP Gravity Vector:\n" << _FgUn);
        }
        //_nUn = _GammaUn * _dx + _FgUn;
        _nUn = _GammaVectorUn + _FgUn;
        return true;        
    } else {
        ROS_ERROR("[updateUnParams_ric_simple/getUncertaintyDynamicsFromService] Failed to call service serverOperationalSpaceDynamics.");
        return false;
    }
}

// 4. CB function for desired acceleration
// Callback for joint states to retrieve joint accelerations
void desiredStateCallback(const smm_control::CustomTcpState::ConstPtr& msg) {
    if (msg->acceleration.size() >= 3) {
        _ddx_d << static_cast<float>(msg->acceleration[0]), 
                 static_cast<float>(msg->acceleration[1]), 
                 static_cast<float>(msg->acceleration[2]);
    } else {
        ROS_WARN("[updateUnParams_ric_simple/desiredStateCallback] Expected at least 3 joint accelerations, received %zu", msg->acceleration.size());
    }
}

// 5. CB Function for the DtLambda
void lambdaTimeDerivativeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);

    if (msg->data.size() != 9) {
        ROS_ERROR("[lambdaTimeDerivativeCallback] Expected 9 elements in the message, but received %zu.", msg->data.size());
        return;
    }

    // Convert the Float64MultiArray data into an Eigen::Matrix3f
    int index = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            _DtLambda(i, j) = msg->data[index++];
        }
    }
}

// 6. CB function that gets the norm of the inverse(Lambda)
void invLambdaNormCallback(const std_msgs::Float32::ConstPtr& msg) {
    _normInvLambda = msg->data;
}

// 7. Calculates the coeffs of the VSC2, p.305 in Book
void calculateAdaptiveCoeffs(ros::NodeHandle& nh) {
    // 1. Get the dynamics
    if (!getDynamicsFromService(nh, false, true, true, true)) {     
        ROS_ERROR("Failed to retrieve operational space matrices.");
        return;
    }
    // 2. Get the "uncertain" dynamics
    if (!getUncertaintyDynamicsFromService(nh, true, true, true)) {     
        ROS_ERROR("Failed to retrieve uncertain operational space matrices.");
        return;
    }
    // 3. M_bar
    M_ij_bar = params.w_margin * (_Lambda - _LambdaUn).cwiseAbs();
    // 4. m_bar
    m_ij_bar = params.w_margin * _DtLambda.cwiseAbs();
    // 5. c_bar
    c_i_bar = params.w_margin * ((_Lambda - _LambdaUn) *_ddx_d).cwiseAbs();
    // 6. n_bar
    n_i_bar = params.w_margin * (_n - _nUn).cwiseAbs();
}

void calculateAdaptiveParams(ros::Publisher& adaptive_params_pub) {
    smm_control::RidoscAdaptiveParams adapt_params_msg;

    // Get the raw vectors
    Eigen::Vector3f raw_pi_adapt_vec =  _normInvLambda * 0.5f * (M_ij_bar * ki_vec); // pi
    Eigen::Vector3f raw_qi_adapt_vec =  _normInvLambda * (M_ij_bar * _lambda0Diag * abs_velocity_error + n_i_bar + c_i_bar); // qi

    // Apply the limit to each element
    for (int i = 0; i < 3; ++i) {
        pi_adapt_vec[i] = std::min(raw_pi_adapt_vec[i], params.pi_limit);
        qi_adapt_vec[i] = std::min(raw_qi_adapt_vec[i], params.qi_limit);
    }

    // Assign to msg data
    adapt_params_msg.pi[0] = static_cast<double>(pi_adapt_vec(0));
    adapt_params_msg.pi[1] = static_cast<double>(pi_adapt_vec(1));
    adapt_params_msg.pi[2] = static_cast<double>(pi_adapt_vec(2));
    adapt_params_msg.qi[0] = static_cast<double>(qi_adapt_vec(0));
    adapt_params_msg.qi[1] = static_cast<double>(qi_adapt_vec(1));
    adapt_params_msg.qi[2] = static_cast<double>(qi_adapt_vec(2));

    // Publish the calculated eta and epsilon values
    adaptive_params_pub.publish(adapt_params_msg);
}

bool loadParameters(ros::NodeHandle& nh, Parameters& params) {
    // Load lambda_0
    if (!nh.getParam("lambda_0", params.lambda_0) || params.lambda_0.size() != 3) {
        ROS_ERROR("[updateUnParams_ric_simple] Failed to load 'lambda_0' or size mismatch (expected 3 elements).");
        return false;
    } else {
        lambda0_vec << params.lambda_0[0], params.lambda_0[1], params.lambda_0[2];
        _lambda0Diag = lambda0_vec.asDiagonal();
    }

    // Load w_margin
    if (!nh.getParam("w_margin", params.w_margin) || params.w_margin <= 1.0f) {
        ROS_ERROR("[updateUnParams_ric_simple] Failed to load 'w_margin'. Must be loaded and greater than 1.0.");
        return false;
    }

    // Load ki
    if (!nh.getParam("ki", params.ki) || params.ki.size() != 3) {
        ROS_ERROR("[updateUnParams_ric_simple] Failed to load 'ki' or size mismatch (expected 3 elements).");
        return false;
    } else {
        ki_vec << params.ki[0], params.ki[1], params.ki[2];
    }

    // Load pi_limit
    if (!nh.getParam("pi_limit", params.pi_limit) || params.pi_limit <= 0.0f) {
        ROS_ERROR("[updateUnParams_ric_simple] Failed to load 'pi_limit'. Must be loaded and greater than 0.");
        return false;
    }

    // Load qi_limit
    if (!nh.getParam("qi_limit", params.qi_limit) || params.qi_limit <= 0.0f) {
        ROS_ERROR("[updateUnParams_ric_simple] Failed to load 'qi_limit'. Must be loaded and greater than 0.");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateUnParams_ric_simple");
    ros::NodeHandle nh;

    if (!loadParameters(nh, params)) {
        ROS_ERROR("Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscribe to topics
    ros::Subscriber error_state_sub = nh.subscribe("/ridosc_error_state", 10, errorStateCallback);
    ros::Subscriber lambda_sub = nh.subscribe("/dtLambda", 10, lambdaTimeDerivativeCallback);
    ros::Subscriber desired_state_sub = nh.subscribe("/tcp_desired_state", 10, desiredStateCallback);
    ros::Subscriber norm_inv_lambda_sub = nh.subscribe("/inverse_lambda_norm", 10, invLambdaNormCallback);
    ros::Subscriber current_tcp_state_sub = nh.subscribe("/tcp_current_state", 10, currentTcpStateCallback);

    // Publisher for calculated pi and qi adapative params
    ros::Publisher adaptive_params_pub = nh.advertise<smm_control::RidoscAdaptiveParams>("/uridosc_adaptive_params", 10); 

    ros::Rate rate(UPDATE_PARAMS_RATE);  // Adjust rate as needed
    while (ros::ok()) {
        ros::spinOnce();
        calculateAdaptiveCoeffs(nh);
        calculateAdaptiveParams(adaptive_params_pub);
        rate.sleep();
    }

    return 0;
}
