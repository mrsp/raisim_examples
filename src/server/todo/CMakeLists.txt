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

if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_release/bin")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_debug/bin")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE})
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG})
else()
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_BINARY_DIR}/examples)
endif()


find_package(qpmad)


find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO pinocchio REQUIRED)
pkg_check_modules(TSID tsid REQUIRED)


include_directories (${EIGEN3_INCLUDE_DIRS} /home/master/catkin_ws/src/whole_body_ik/include   /home/master/catkin_ws/src/lipm_control/include /home/master/catkin_ws/src/lipm_motion/include ${PINOCCHIO_INCLUDE_DIRS}
${qpmad_INCLUDE_DIRS} ${TSID_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS} tsid)


#==
# Build Targets
#==



function(create_executable app_name file_name)
    add_executable(${app_name} ${file_name})
    set_target_properties(${app_name} PROPERTIES MACOSX_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/mac/lib")
    if(WIN32)
        target_link_libraries(${app_name} PUBLIC raisim::raisim Ws2_32 Winmm)
        target_compile_options(${app_name} PRIVATE "/MP")
    else()
        target_link_libraries(${app_name} PUBLIC raisim::raisim pthread)

    endif()
    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endfunction()

# benchmark examples
create_executable(anymal_speed_test src/benchmark/anymal.cpp)

# server examples
create_executable(primitives src/server/primitives.cpp)
create_executable(meshes src/server/meshes.cpp)
create_executable(laikago src/server/laikago.cpp)
create_executable(balls src/server/balls.cpp)
#create_executable(atlas src/server/atlas.cpp )


add_executable(atlas src/server/atlas.cpp  /home/master/catkin_ws/src/whole_body_ik/src/pin_wrapper.cpp /home/master/catkin_ws/src/lipm_motion/lipm.cpp /home/master/catkin_ws/src/lipm_motion/zmpPlanner.cpp /home/master/catkin_ws/src/lipm_motion/dcmPlanner.cpp /home/master/catkin_ws/src/lipm_motion/LIPMPlanner.cpp
/home/master/catkin_ws/src/lipm_motion/dcmDynamics.cpp /home/master/catkin_ws/src/lipm_motion/LIPMDynamics.cpp /home/master/catkin_ws/src/lipm_control/src/RobotParameters.cpp /home/master/catkin_ws/src/lipm_control/src/ZMPDistributor.cpp /home/master/catkin_ws/src/lipm_control/src/postureStabilizer.cpp /home/master/catkin_ws/src/lipm_control/src/FootPolygon.cpp)
target_link_libraries(atlas PUBLIC raisim::raisim pthread ${EIGEN3_LIBRARIES} ${PINOCCHIO_LIBRARIES} tsid)
target_compile_definitions(atlas PRIVATE ${PINOCCHIO_CFLAGS_OTHER} ${TSID_CFLAGS_OTHER})
message(STATUS ${TSID_LIBRARIES})
message(STATUS ${TSID_LIBRARY_DIRS})
message(STATUS ${PINOCCHIO_LIBRARIES})
message(STATUS ${PINOCCHIO_LIBRARY_DIRS})

create_executable(heightmap src/server/heightmap.cpp)
create_executable(robots src/server/robots.cpp)
create_executable(materials src/server/material.cpp)
create_executable(heightMapUsingPng src/server/heightMapUsingPng.cpp)
create_executable(wheeledRobot src/server/wheeledRobot.cpp)
create_executable(newtonsCradle src/server/newtonsCradle.cpp)
create_executable(visualObjects src/server/visualObjects.cpp)
create_executable(rayDemo src/server/rayDemo.cpp)
create_executable(rayDemo2 src/server/rayDemo2.cpp)
create_executable(anymals src/server/anymals.cpp)
create_executable(anymal_stress_test src/server/anymals_stress_test.cpp)
create_executable(compound src/server/compound.cpp)
create_executable(kinova src/server/kinova.cpp)
create_executable(cartPole src/server/cartPole.cpp)
create_executable(springTest src/server/springTest.cpp)

# xml reader
create_executable(xmlRader src/xml/xmlReader.cpp)