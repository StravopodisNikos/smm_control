cmake_minimum_required(VERSION 3.0.2)
project(smm_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  controller_interface
  controller_manager
  geometry_msgs
  hardware_interface
  pluginlib
  roscpp
  rospy
  sensor_msgs
  smm_screws
  std_msgs
  message_generation # [29-7-24] Added for custom controllers messages
  dynamic_reconfigure
  kdl_parser
  tf2
  tf2_ros
  visualization_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# Find orocos_kdl (for KDL)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
add_message_files(
  FILES
  IdoscError.msg 
  IdoscDesired.msg 
  IdoscCurrent.msg 
  IdoscGainMatrices.msg
  IdoscControlOutput.msg
  IdoscTorques.msg
  CustomJointState.msg
  FasmcError.msg
  FasmcFuzzyParams.msg
  FasmcAdaptiveParams.msg
  FasmcTorques.msg
  RobotTorques.msg
  CustomTcpState.msg
  RidoscAdaptiveParams.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
add_service_files(
  FILES
  GetJacobians.srv
  GetDynamics.srv
  GetSpatialJacobians.srv
  GetTcpStateTfNum.srv
  GetOperationalSpaceDynamics.srv
  GetOperationalSpaceMatrices.srv
  GetUncertaintyDynamics.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   control_msgs#   geometry_msgs#   sensor_msgs#   std_msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs # for IdoscError/Current/Desired
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )
generate_dynamic_reconfigure_options(
  cfg/IdoscGain.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES idosc # [30-7-24] Added for using idosc class as ROS library
  CATKIN_DEPENDS control_msgs controller_interface controller_manager geometry_msgs hardware_interface pluginlib roscpp rospy sensor_msgs smm_screws std_msgs message_runtime dynamic_reconfigure kdl_parser tf2 tf2_ros visualization_msgs
  #DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(
#  include # [30-7-24] Added for using idosc class as ROS library
#  ${catkin_INCLUDE_DIRS}
#)
include_directories(
  include
  ~/cpp_ws/libraries/eigen-3.4.0
  ~/cpp_ws/libraries/odeint-v2-master
  ~/cpp_ws/libraries/fuzzylite/include
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library
add_library(fuzzylite_lib SHARED IMPORTED)
set_target_properties(fuzzylite_lib PROPERTIES
    IMPORTED_LOCATION ~/cpp_ws/libraries/fuzzylite/build/bin/libfuzzylite.so
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/smm_control.cpp
# )
#add_library idosc
add_library(smm_control_lib # [31-7-24] set idosc as library
src/idosc.cpp
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/smm_control_node.cpp)
## IDOSC
add_executable(initialize_robot_shared src/initialize_robot_shared.cpp)
add_executable(updateJointState_idosc_simple src/updateJointState_idosc_simple.cpp)
add_executable(updateCartesianState_idosc_simple src/updateCartesianState_idosc_simple.cpp)
add_executable(publishCartesianPath_idosc_simple src/publishCartesianPath_idosc_simple.cpp)
add_executable(publishCartesianPathLoop_idosc_simple src/publishCartesianPathLoop_idosc_simple.cpp)
add_executable(updateErrorState_idosc_simple src/updateErrorState_idosc_simple.cpp)
#add_executable(updateToolJacobians src/updateToolJacobians.cpp) ##DEPRECATED##
add_executable(serverToolJacobians src/serverToolJacobians.cpp)
add_executable(updateGains_idosc_simple src/updateGains_idosc_simple.cpp)
add_executable(updateControlOutput_idosc_simple src/updateControlOutput_idosc_simple.cpp)
add_executable(serverRobotDynamics src/serverRobotDynamics.cpp)
add_executable(serverRobotDynamicsKDL src/serverRobotDynamicsKDL.cpp)
add_executable(updateTorques_idosc_simple src/updateTorques_idosc_simple.cpp)
add_executable(robot_effort_publisher src/robot_effort_publisher.cpp)
## FASMC
add_executable(updateJointState_fasmc_simple src/updateJointState_fasmc_simple.cpp)
add_executable(publishJointPathLoop_fasmc_simple src/publishJointPathLoop_fasmc_simple.cpp)
add_executable(updateErrorState_fasmc_simple src/updateErrorState_fasmc_simple.cpp)
add_executable(updateSlidingState_fasmc_simple src/updateSlidingState_fasmc_simple.cpp)
add_executable(updateFuzzyParams_fasmc_simple src/updateFuzzyParams_fasmc_simple.cpp)
add_executable(updateParams_fasmc_simple src/updateParams_fasmc_simple.cpp)
add_executable(updateRobustTorque_fasmc_simple src/updateRobustTorque_fasmc_simple.cpp)
add_executable(updateDynamicsTorque_fasmc_simple src/updateDynamicsTorque_fasmc_simple.cpp)
add_executable(updateTorqueCommand_fasmc_simple src/updateTorqueCommand_fasmc_simple.cpp)
add_executable(robot_torque_command_publisher src/robot_torque_command_publisher.cpp)
## RIDOSC
add_executable(updateJointState_simple src/updateJointState_simple.cpp)
add_executable(serverSpatialJacobians src/serverSpatialJacobians.cpp)
add_executable(updateCartesianState_ridosc_simple src/updateCartesianState_ridosc_simple.cpp)
add_executable(serverTcpStateTfNum src/serverTcpStateTfNum.cpp)
add_executable(updateDesiredState_ridosc_simple src/updateDesiredState_ridosc_simple.cpp)
add_executable(updateErrorState_ridosc_simple src/updateErrorState_ridosc_simple.cpp)
add_executable(updateParams_ridosc_simple src/updateParams_ridosc_simple.cpp)
add_executable(updateSlidingState_ridosc_simple src/updateSlidingState_ridosc_simple.cpp)
add_executable(updateRobustTorque_ridosc_simple src/updateRobustTorque_ridosc_simple.cpp)
add_executable(serverOperationalSpaceDynamics src/serverOperationalSpaceDynamics.cpp)
add_executable(updateDynamicsTorque_ridosc_simple src/updateDynamicsTorque_ridosc_simple.cpp)
add_executable(updateTorqueCommand_ridosc_simple src/updateTorqueCommand_ridosc_simple.cpp)
add_executable(serverOperationalSpaceMatrices src/serverOperationalSpaceMatrices.cpp)
add_executable(updateUnParams_ridosc_simple src/updateUncertaintyParams_ridosc_simple.cpp)
## HRIDOSC
add_executable(updateParams_hridosc_simple src/updateParams_hridosc_simple.cpp)
add_executable(updateRobustTorque_hridosc_simple src/updateRobustTorque_hridosc_simple.cpp)
add_executable(updateTorqueCommand_hridosc_simple src/updateTorqueCommand_hridosc_simple.cpp)
## URIDOSC
add_executable(updateRobustTorque_uridosc_simple src/updateRobustTorque_uridosc_simple.cpp)
## UNCERTAINTY DYNAMICS
add_executable(updateUncertaintyFactor src/updateUncertaintyFactor.cpp)
add_executable(serverUncertaintyMatrices src/serverUncertainOperationalSpaceMatrices.cpp)
add_executable(updateDtLambda src/updateDtLambda.cpp)
add_executable(updateInverseLambda src/updateInverseLambda.cpp)
## RIC
add_executable(updateImpedanceTorque_ric_simple src/updateImpedanceTorque_ric_simple.cpp)
## BUILT-IN DYNAMICS AND SIMPLE GRAVITY COMPENSATION
add_executable(publishSingleJointGoal src/publishSingleJointGoal.cpp)
add_executable(publishSingleTcpGoal src/publishSingleTcpGoal.cpp)
add_executable(stabilizeRobot src/stabilizeRobot.cpp)
## Visual debugging tools
add_executable(visualizeTcpGoal src/visualizeTcpGoal.cpp)
add_executable(visualizeTcpStates src/visualizeTcpStates.cpp)
## ENVIRONMENT INTERACTION
add_executable(ext_force_torque_publisher src/ext_force_torque_publisher.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## IDOSC
add_dependencies(updateCartesianState_idosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publishCartesianPath_idosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publishCartesianPathLoop_idosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateErrorState_idosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverToolJacobians ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateGains_idosc_simple ${PROJECT_NAME}_gencfg)
add_dependencies(updateControlOutput_idosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverRobotDynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverRobotDynamicsKDL ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateTorques_idosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_effort_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## FASMC
add_dependencies(updateJointState_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publishJointPathLoop_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateErrorState_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateSlidingState_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateFuzzyParams_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateParams_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateRobustTorque_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateDynamicsTorque_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateTorqueCommand_fasmc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_torque_command_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## RIDOSC
add_dependencies(updateJointState_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverSpatialJacobians ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateCartesianState_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverTcpStateTfNum ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateDesiredState_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateErrorState_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateParams_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateSlidingState_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateRobustTorque_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverOperationalSpaceDynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateDynamicsTorque_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateTorqueCommand_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverOperationalSpaceMatrices ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateUnParams_ridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## HRIDOSC
add_dependencies(updateParams_hridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateRobustTorque_hridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateTorqueCommand_hridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## URIDOSC
add_dependencies(updateRobustTorque_uridosc_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## RIC
add_dependencies(updateImpedanceTorque_ric_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## UNCERTAINTY DYNAMICS
add_dependencies(updateUncertaintyFactor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serverUncertaintyMatrices ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateDtLambda ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(updateInverseLambda ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## BUILT-IN DYNAMICS AND SIMPLE GRAVITY COMPENSATION && DESIRED GOALs
add_dependencies(publishSingleJointGoal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publishSingleTcpGoal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(stabilizeRobot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Visual debugging tools
add_dependencies(visualizeTcpGoal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(visualizeTcpStates ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## ENVIRONMENT INTERACTION
add_dependencies(ext_force_torque_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(idosc
target_link_libraries(smm_control_lib
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(initialize_robot_shared
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(updateJointState_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(updateCartesianState_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(publishCartesianPath_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(publishCartesianPathLoop_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateErrorState_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(serverToolJacobians
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateGains_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateControlOutput_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(serverRobotDynamics
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(serverRobotDynamicsKDL
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateTorques_idosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(robot_effort_publisher
 ${catkin_LIBRARIES}
 robot_shared 
)
## FASMC
target_link_libraries(updateJointState_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(publishJointPathLoop_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateErrorState_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateSlidingState_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateFuzzyParams_fasmc_simple 
 ${catkin_LIBRARIES}
 robot_shared
 /home/nikos/cpp_ws/libraries/fuzzylite/build/bin/libfuzzylite.so
)
target_link_libraries(updateParams_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateRobustTorque_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateDynamicsTorque_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateTorqueCommand_fasmc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(robot_torque_command_publisher
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(publishSingleJointGoal
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(publishSingleTcpGoal
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(stabilizeRobot
 ${catkin_LIBRARIES}
 robot_shared 
)
## RIDOSC
target_link_libraries(updateJointState_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(serverSpatialJacobians
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateCartesianState_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(serverTcpStateTfNum
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(updateDesiredState_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
) 
target_link_libraries(updateErrorState_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateParams_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateSlidingState_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateRobustTorque_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(serverOperationalSpaceDynamics
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateDynamicsTorque_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateTorqueCommand_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(serverOperationalSpaceMatrices
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateUnParams_ridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
## HRIDOSC
target_link_libraries(updateParams_hridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateRobustTorque_hridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateTorqueCommand_hridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
## URIDOSC
target_link_libraries(updateRobustTorque_uridosc_simple
 ${catkin_LIBRARIES}
 robot_shared 
)
## RIC
target_link_libraries(updateImpedanceTorque_ric_simple
 ${catkin_LIBRARIES}
)
## UNCERTAINTY DYNAMICS
target_link_libraries(updateUncertaintyFactor
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(serverUncertaintyMatrices
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateDtLambda
 ${catkin_LIBRARIES}
 robot_shared 
)
target_link_libraries(updateInverseLambda
 ${catkin_LIBRARIES}
 robot_shared 
)
## VISUAL DEBUGGING TOOLS
target_link_libraries(visualizeTcpGoal ${catkin_LIBRARIES})
target_link_libraries(visualizeTcpStates
 ${catkin_LIBRARIES}
 robot_shared 
)

## ENVIRONMENT INTERACTION
target_link_libraries(ext_force_torque_publisher ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
#install(TARGETS idosc
install(TARGETS smm_control_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
install(DIRECTORY include/smm_control/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_smm_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
