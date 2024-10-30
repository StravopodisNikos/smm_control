#ifndef TIMING_H
#define TIMING_H

// Define low testing rates in Hz
constexpr double UPDATE_SLIDING_SURFACE_RATE        = 10.0;    
constexpr double UPDATE_ERROR_STATE_RATE            = 10.0;
constexpr double UPDATE_PARAMS_RATE 				= 10.0;             
constexpr double UPDATE_FUZZY_PARAMS_RATE 			= 5.0;       
constexpr double UPDATE_DYNAMICS_TORQUE_RATE 		= 5.0;    
constexpr double UPDATE_ROBUST_TORQUE_RATE 			= 5.0;      
constexpr double UPDATE_TORQUE_COMMAND_RATE 		= 5.0;     
constexpr double UPDATE_JOINT_PATH_LOOP_RATE 		= 10.0;           
constexpr double UPDATE_SERVER_DYNAMICS_RATE 		= 10.0; 
constexpr double UPDATE_JOINT_STATE_RATE 		    = 10.0;

#endif // TIMING_H
