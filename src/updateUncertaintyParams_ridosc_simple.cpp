#include <ros/ros.h>
#include <smm_control/IdoscError.h>
#include <smm_control/RidoscAdaptiveParams.h> 
#include <smm_control/GetOperationalSpaceMatrices.h> 
#include <smm_control/GetUncertaintyDynamics.h> 
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "smm_control/timing.h"

// Define matrices and vectors 
Eigen::Matrix3f _Lambda;                         
Eigen::Vector3f _GammaVector;
Eigen::Vector3f _Fg;                             
Eigen::Matrix3f _LambdaUn;                         
Eigen::Vector3f _GammaVectorUn;
Eigen::Vector3f _FgUn;          

// Initialization only
std::vector<double> qi_adapt(3, 0.01);   
std::vector<double> pi_adapt(3, 0.01); 
boost::array<double, 3> position_error = {0.0, 0.0, 0.0};
boost::array<double, 3> velocity_error = {0.0, 0.0, 0.0};

ros::Publisher ridosc_adaptive_params_pub;

// Fixed params loaded from yaml ridosc/adaptive_params.yaml
struct Parameters {
    std::vector<float> lambda_0; // lambda_0: [17.5, 17.50, 17.5]
    float w_margin;              // w_margin: 1.5
    std::vector<float> ki;       // ki: [1.0, 1.0, 1.0]
    float pi_limit;              // pi_limit: 10.0
    float qi_limit;              // qi_limit: 10.0
};

// 1. Callback for /ridosc_error_state
void errorStateCallback(const smm_control::IdoscError::ConstPtr& msg) {
    // Extract position error from the pose_error field
    velocity_error[0] = msg->pose_error.position.x;
    velocity_error[1] = msg->pose_error.position.y;
    velocity_error[2] = msg->pose_error.position.z;

    // Log the extracted position error (optional for debugging)
    ROS_INFO("[updateUnParams_ridosc_simple/errorStateCallback] Position Error: x=%.3f, y=%.3f, z=%.3f", 
             velocity_error[0], velocity_error[1], velocity_error[2]);
}

// 2. Function to call the service and retrieve current Operational Dynamic matrices
bool getDynamicsFromService(ros::NodeHandle& nh, bool get_JacobianMatrix, bool get_LambdaMatrix, bool get_GammaVector, bool get_FgVector) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetOperationalSpaceMatrices>("GetOperationalSpaceMatrices");
    smm_control::GetOperationalSpaceMatrices srv;
 
    // Set flags in the request
    srv.request.get_op_jacobian = get_JacobianMatrix;
    srv.request.get_Lambda = get_LambdaMatrix;
    srv.request.get_Gamma_OSD_Vector = get_GammaVector;
    srv.request.get_Fg = get_FgVector;

    if (client.call(srv)) {
        if (get_LambdaMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Lambda(i, j) = srv.response.Lambda[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Lambda Matrix:\n" << _Lambda);
        }
        if (get_GammaVector) {
            for (int i = 0; i < 3; i++) {
                _GammaVector(i) = srv.response.Gamma_OSD_Vector[i];
            }
           // ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] TCP Gamma Vector:\n" << _GammaVector);
        }   
        if (get_FgVector) {
            for (int i = 0; i < 3; i++) {
                _Fg(i) = srv.response.Fg[i];
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] TCP Gravity Vector:\n" << _Fg);
        }
        return true;        
    } else {
        ROS_ERROR("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Failed to call service serverOperationalSpaceDynamics.");
        return false;
    }
}

// 3. Function to call the service and retrieve Modified Operational Dynamic matrices based on Uncertainty 
bool getUncertaintyDynamicsFromService(ros::NodeHandle& nh, bool get_LambdaUnMatrix, bool get_GammaUnVector, bool get_FgUnVector) {
    ros::ServiceClient client = nh.serviceClient<smm_control::GetUncertaintyDynamics>("GetUncertaintyDynamics");
    smm_control::GetUncertaintyDynamics srv;
 
    // Set flags in the request
    srv.request.get_Lambda_un = get_LambdaUnMatrix;
    srv.request.get_Gamma_un = get_GammaUnVector;
    srv.request.get_Fg_un = get_FgUnVector;

    if (client.call(srv)) {
        if (get_LambdaUnMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _LambdaUn(i, j) = srv.response.Lambda_un[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Lambda Matrix:\n" << _Lambda);
        }
        if (get_GammaUnVector) {
            for (int i = 0; i < 3; i++) {
                _GammaVectorUn(i) = srv.response.Gamma_un[i];
            }
           // ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] TCP Gamma Vector:\n" << _GammaVector);
        }   
        if (get_FgUnVector) {
            for (int i = 0; i < 3; i++) {
                _FgUn(i) = srv.response.Fg_un[i];
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] TCP Gravity Vector:\n" << _Fg);
        }
        return true;        
    } else {
        ROS_ERROR("[updateDynamicsTorque_ridosc_simple/getUncertaintyDynamicsFromService] Failed to call service serverOperationalSpaceDynamics.");
        return false;
    }
}

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
    // 
}

void calculateAdaptiveParams(ros::Publisher& adaptive_params_pub) {
    smm_control::RidoscAdaptiveParams adapt_params_msg;

    for (int i = 0; i < 3; ++i) {
        double abs_velocity_error = std::abs(velocity_error[i]);
        //adapt_params_msg.pi[i] = 
        //adapt_params_msg.qi[i] = 
    }

    // Publish the calculated eta and epsilon values
    adaptive_params_pub.publish(adapt_params_msg);
}

bool loadParameters(ros::NodeHandle& nh, Parameters& params) {
    // Load lambda_0
    if (!nh.getParam("lambda_0", params.lambda_0) || params.lambda_0.size() != 3) {
        ROS_ERROR("[updateUnParams_ridosc_simple] Failed to load 'lambda_0' or size mismatch (expected 3 elements).");
        return false;
    }

    // Load w_margin
    if (!nh.getParam("w_margin", params.w_margin) || params.w_margin <= 1.0f) {
        ROS_ERROR("[updateUnParams_ridosc_simple] Failed to load 'w_margin'. Must be loaded and greater than 1.0.");
        return false;
    }

    // Load ki
    if (!nh.getParam("ki", params.ki) || params.ki.size() != 3) {
        ROS_ERROR("[updateUnParams_ridosc_simple] Failed to load 'ki' or size mismatch (expected 3 elements).");
        return false;
    }

    // Load pi_limit
    if (!nh.getParam("pi_limit", params.pi_limit) || params.pi_limit <= 0.0f) {
        ROS_ERROR("[updateUnParams_ridosc_simple] Failed to load 'pi_limit'. Must be loaded and greater than 0.");
        return false;
    }

    // Load qi_limit
    if (!nh.getParam("qi_limit", params.qi_limit) || params.qi_limit <= 0.0f) {
        ROS_ERROR("[updateUnParams_ridosc_simple] Failed to load 'qi_limit'. Must be loaded and greater than 0.");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "updateUnParams_ridosc_simple");
    ros::NodeHandle nh;

    Parameters params;
    if (!loadParameters(nh, params)) {
        ROS_ERROR("Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscribe to topics
    ros::Subscriber error_state_sub = nh.subscribe("/ridosc_error_state", 10, errorStateCallback);

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
