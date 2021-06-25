cmake_minimum_required(VERSION 3.5)
project(raisim_examples LANGUAGES CXX)


#==
# Dependencies
#==
add_compile_options(-std=c++11)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")
find_package(Boost REQUIRED COMPONENTS system)



find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(qpmad)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO pinocchio REQUIRED)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})



include_directories (${EIGEN3_INCLUDE_DIRS} /home/master/catkin_ws/src/whole_body_ik/include   /home/master/catkin_ws/src/lipm_control/include /home/master/catkin_ws/src/lipm_motion/include ${PINOCCHIO_INCLUDE_DIRS}
${qpmad_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})


#==
# Build Targets
#==

add_executable(naoIK src/server/nao.cpp  /home/master/catkin_ws/src/whole_body_ik/src/pin_wrapper.cpp /home/master/catkin_ws/src/lipm_motion/lipm.cpp /home/master/catkin_ws/src/lipm_motion/zmpPlanner.cpp /home/master/catkin_ws/src/lipm_motion/dcmPlanner.cpp /home/master/catkin_ws/src/lipm_motion/LIPMPlanner.cpp
/home/master/catkin_ws/src/lipm_motion/dcmDynamics.cpp /home/master/catkin_ws/src/lipm_motion/LIPMDynamics.cpp /home/master/catkin_ws/src/lipm_control/src/RobotParameters.cpp /home/master/catkin_ws/src/lipm_control/src/ZMPDistributor.cpp /home/master/catkin_ws/src/lipm_control/src/postureStabilizer.cpp /home/master/catkin_ws/src/lipm_control/src/FootPolygon.cpp)
target_link_libraries(naoIK PUBLIC raisim::raisim pthread ${EIGEN3_LIBRARIES} ${PINOCCHIO_LIBRARIES})
target_compile_definitions(naoIK PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

