#ifndef TIMING_H
#define TIMING_H

// Define low testing rates in Hz
constexpr double UPDATE_SLIDING_SURFACE_RATE        = 1000.0;    // 1000
constexpr double UPDATE_ERROR_STATE_RATE            = 1000.0;    // 1000 
constexpr double UPDATE_PARAMS_RATE 				= 100.0;     // 100         
constexpr double UPDATE_FUZZY_PARAMS_RATE 			= 100.0;     // 100 
constexpr double UPDATE_DYNAMICS_TORQUE_RATE 		= 10.0;    // 1000 
constexpr double UPDATE_ROBUST_TORQUE_RATE 			= 1000.0;    // 1000
constexpr double UPDATE_TORQUE_COMMAND_RATE 		= 100.0;     // 100
constexpr double UPDATE_JOINT_PATH_LOOP_RATE 		= 1.0;       //       
constexpr double UPDATE_SERVER_DYNAMICS_RATE 		= 10.0;     // 100
constexpr double UPDATE_SERVER_JACOBIANS_RATE       = 10.0;     // 100
constexpr double UPDATE_JOINT_STATE_RATE 		    = 1000.0;    // 1000

constexpr double UPDATE_DESIRED_STATE_RATE          = 1000.0;    // 1000
constexpr double IKP_LOOP_RATE     		            = 1.0;     // 

#endif // TIMING_H
