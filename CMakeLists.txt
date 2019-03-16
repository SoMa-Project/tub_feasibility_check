cmake_minimum_required(VERSION 2.8.3)
project(tub_feasibility_check)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations -Wno-zero-as-null-pointer-constant -fno-inline-functions -fpermissive")

add_definitions(-DEIGEN_DONT_ALIGN)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    message_generation
    geometry_msgs
    shape_msgs
    tf
    tf_conversions
    eigen_conversions
)

message(${PROJECT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/contact-motion-planning/Modules/")

add_subdirectory(contact-motion-planning EXCLUDE_FROM_ALL)
set(ROBLIB_INCLUDE_DIRS contact-motion-planning/src)

find_package(Boost REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})
include_directories(${CMAKE_BINARY_DIR})

find_package(Bullet REQUIRED)
find_package(SoQt REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(Coin REQUIRED)

include_directories(include
${Boost_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${OPENGL_INCLUDE_DIR}
${QT_INCLUDES}
${SOQT_INCLUDE_DIRS}
${BULLET_INCLUDE_DIRS}
${LIBXML2_INCLUDE_DIRS}
${COIN_INCLUDE_DIRS}
${ROBLIB_INCLUDE_DIRS}
    )

add_service_files(
    FILES
    CheckKinematics.srv
    CheckKinematicsTabletop.srv
    CheckSurfaceGrasp.srv
    VisualizeTrajectory.srv
)

add_message_files(
    FILES
    BoundingBoxWithPose.msg
    AllowedCollision.msg
    SurfaceGraspPregraspManifold.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    shape_msgs
    geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)

#catkin_add_gtest(workspace_checker_sampler_test test/test_workspace_checker_and_sampler.cpp src/workspace_samplers.cpp
#    src/workspace_checkers.cpp)
#target_link_libraries(workspace_checker_sampler_test
#        rlplan
#        rlkin
#        rlsg)

catkin_add_gtest(surface_pregrasp_manifolds_test test/test_surface_pregrasp_manifolds.cpp
    src/manifold.cpp src/manifolds/circular_manifold.cpp
    src/manifolds/elongated_manifold.cpp
    src/manifolds/wall_grasp_manifold.cpp)
target_link_libraries(surface_pregrasp_manifolds_test
        rlplan
        rlkin
        rlsg)

if(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND )
    set(SRCS
        src/mainwindow.cpp
        src/kinematics_check.cpp
        src/Viewer.cpp
        src/ifco_scene.cpp
        src/service_worker.cpp
        src/jacobian_controller.cpp
        src/collision_specification.cpp
        src/usecase_scene.cpp
        src/tabletop_scene.cpp
        src/manifold.cpp
        src/manifolds/circular_manifold.cpp
        src/manifolds/elongated_manifold.cpp
        src/manifolds/old_manifold.cpp
        src/manifolds/wall_grasp_manifold.cpp
        src/check_kinematics_parameters.cpp
    )

qt4_wrap_ui(UI_HEADERS src/mainwindow.ui)

    qt4_wrap_cpp(
        MOC_SRCS
        src/Viewer.h
        src/service_worker.h
        src/usecase_scene.h
        src/jacobian_controller.h
        src/mainwindow.h
        OPTIONS
        -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
    )

    add_executable(
        tub_feasibility_check
        ${SRCS}
        ${MOC_SRCS}
        ${UI_HEADERS}
    )

    target_compile_definitions(
        tub_feasibility_check
        PUBLIC
        ${QT_DEFINITIONS}
        ${SOQT_DEFINITIONS}
        ${COIN_DEFINITIONS}
    )

    target_include_directories(
        tub_feasibility_check
        PUBLIC
        src/
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIR}
        ${QT_INCLUDES}
        ${SOQT_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIRS}
        ${LIBXML2_INCLUDE_DIRS}
        ${COIN_INCLUDE_DIRS}
        ${ROBLIB_INCLUDE_DIRS}
    )


    target_link_libraries(
        tub_feasibility_check
        rlplan
        rlkin
        rlsg
        ${catkin_LIBRARIES}
        ${ROS_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${QT_LIBRARIES}
        ${SOQT_LIBRARIES}
        ${BULLET_LIBRARIES}
        ${LIBXML2_LIBRARIES}
        ${COIN_LIBRARIES}
    )

add_dependencies(tub_feasibility_check tub_feasibility_check_generate_messages_cpp)

endif(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND)
