#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include "smm_control/GetUncertaintyImpedanceDynamics.h"
#include "smm_control/GetImpedanceDynamics.h"
#include "smm_control/timing.h"

// Global variables for dynamics matrices and uncertainty factor
Eigen::Matrix3f _Jop = Eigen::Matrix3f::Zero(); // not loaded here
Eigen::Matrix3f _Lambda = Eigen::Matrix3f::Zero();
Eigen::Matrix3f _Gamma = Eigen::Matrix3f::Zero();
Eigen::Vector3f _GammaVector = Eigen::Vector3f::Zero();
Eigen::Vector3f _Fg = Eigen::Vector3f::Zero();
float uncertainty_factor = 1.0f; // Default uncertainty factor
float lambda_uncertainty;
float gamma_uncertainty;
float gravity_uncertainty;
float lambda_scale;
float gamma_scale;
float gravity_scale;

// Callback for the uncertainty factor
void uncertaintyFactorCallback(const std_msgs::Float32::ConstPtr& msg) {
    uncertainty_factor = msg->data;
}

// Modify uncertainty factors for each matrix
void setUncertaintyFactors() {
    lambda_uncertainty  = lambda_scale * uncertainty_factor;
    gamma_uncertainty   = gamma_scale * uncertainty_factor;
    gravity_uncertainty = gravity_scale * uncertainty_factor;
}

// Function to recalculate matrices based on uncertainty
void applyUncertainty(Eigen::Matrix3f& Lambda, Eigen::Vector3f& Gamma, Eigen::Vector3f& Fg) {
    Lambda *= (1 + lambda_uncertainty);
    Gamma  *= (1 + gamma_uncertainty);
    Fg     *= (1 + gravity_uncertainty);
}

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
            //ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Lambda Matrix:\n" << _Lambda);
        }
        if (get_JacobianMatrix) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    _Jop(i, j) = srv.response.op_jacobian[i * 3 + j];
                }
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] Operational Jacobian Matrix:\n" << _Gamma);
        }
        if (get_GammaMatrix) {
            for (int i = 0; i < 3; i++) {
                //for (int j = 0; j < 3; j++) {
                    _GammaVector(i) = srv.response.Gamma_imp[i];
                //}
            }
           // ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] TCP Gamma Vector:\n" << _GammaVector);
        }   

        if (get_FgVector) {
            for (int i = 0; i < 3; i++) {
                _Fg(i) = srv.response.Fg_imp[i];
            }
            //ROS_INFO_STREAM("[updateDynamicsTorque_ridosc_simple/getDynamicsFromService] TCP Gravity Vector:\n" << _Fg);
        }
        return true;        
    } else {
        ROS_ERROR("[serverUncertaintyImpedanceDynamics/getDynamicsFromService] Failed to call service.");
        return false;
    }
}

// Service callback
bool handleGetUncertaintyDynamics(smm_control::GetUncertaintyImpedanceDynamics::Request &req,
                                   smm_control::GetUncertaintyImpedanceDynamics::Response &res,
                                   ros::NodeHandle& nh) {

    // Call the existing service to get the current dynamics
    if (!getDynamicsFromService(nh, false, req.get_Lambda_un,  req.get_Gamma_un, req.get_Fg_un)) {
        ROS_ERROR("[serverUncertaintyImpedanceDynamics] Failed to get dynamics from service.");
        return false;
    }

    // set response to same matrices for initialization only
    Eigen::Matrix3f Lambda_uncertainty = _Lambda;
    Eigen::Vector3f Gamma_uncertainty = _GammaVector;
    Eigen::Vector3f Fg_uncertainty = _Fg;

    // modify response based on uncertainty
    setUncertaintyFactors();
    applyUncertainty(Lambda_uncertainty, Gamma_uncertainty, Fg_uncertainty);

    // Populate response
    if (req.get_Lambda_un) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                res.Lambda_un[i * 3 + j] = Lambda_uncertainty(i, j);
            }
        }
    }
    if (req.get_Gamma_un) {
        for (int i = 0; i < 3; i++) {
            //for (int j = 0; j < 3; j++) {
                //res.Gamma_un[i * 3 + j] = Gamma_uncertainty(i, j);
                res.Gamma_un[i] = Gamma_uncertainty(i);
            //}
        }
    }
    if (req.get_Fg_un) {
        for (int i = 0; i < 3; i++) {
            res.Fg_un[i] = Fg_uncertainty(i);
        }
    }

    return true;
}

bool loadParameters(ros::NodeHandle& nh, float& lambda_un, float& gamma_un, float& fgrav_un) {
    // Load parameters with default values
    if (!nh.param<float>("lambda_un", lambda_un, 0.5f)) {
        ROS_WARN("[loadParameters] Failed to load 'lambda_un'. Using default value: 0.5");
    }
    if (!nh.param<float>("gamma_un", gamma_un, 0.25f)) {
        ROS_WARN("[loadParameters] Failed to load 'gamma_un'. Using default value: 0.25");
    }
    if (!nh.param<float>("fgrav_un", fgrav_un, 0.1f)) {
        ROS_WARN("[loadParameters] Failed to load 'fgrav_un'. Using default value: 0.1");
    }

    // Check bounds
    if (lambda_un < 0.0f || lambda_un > 1.0f) {
        ROS_ERROR("[loadParameters] 'lambda_un' is out of bounds [0, 1]: %f", lambda_un);
        return false;
    }
    if (gamma_un < 0.0f || gamma_un > 1.0f) {
        ROS_ERROR("[loadParameters] 'gamma_un' is out of bounds [0, 1]: %f", gamma_un);
        return false;
    }
    if (fgrav_un < 0.0f || fgrav_un > 1.0f) {
        ROS_ERROR("[loadParameters] 'fgrav_un' is out of bounds [0, 1]: %f", fgrav_un);
        return false;
    }

    ROS_INFO("[serverUncertaintyImpedanceDynamics/loadParameters] Loaded parameters: lambda_un=%f, gamma_un=%f, fgrav_un=%f", 
             lambda_un, gamma_un, fgrav_un);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serverUncertaintyImpedanceDynamics");
    ros::NodeHandle nh;

    if (!loadParameters(nh, lambda_scale, gamma_scale, gravity_scale)) {
        ROS_ERROR("[serverUncertaintyImpedanceDynamics] Failed to load parameters. Exiting.");
        return -1;
    }

    // Subscriber for uncertainty factor
    ros::Subscriber uncertainty_sub = nh.subscribe("/uncertainty_factor", 10, uncertaintyFactorCallback);

    // Advertise the GetUncertaintyImpedanceDynamics service
    ros::ServiceServer service = nh.advertiseService<smm_control::GetUncertaintyImpedanceDynamics::Request,
                                                     smm_control::GetUncertaintyImpedanceDynamics::Response>(
        "GetUncertaintyImpedanceDynamics",
        boost::bind(handleGetUncertaintyDynamics, _1, _2, boost::ref(nh))
    );

    ros::Rate loop_rate(UPDATE_UNCERTAINTY_DYNAMICS_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
