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

find_package(Bullet REQUIRED)
find_package(SoQt REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(Coin REQUIRED)

enable_testing()


add_service_files(
    FILES
    CheckKinematics.srv
    CerrtExample.srv
)

add_message_files(
    FILES
    BoundingBoxWithPose.msg
    AllowedCollision.msg
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


if(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND )
    set(SRCS
        src/MainWindow.cpp
        src/kinematics_check.cpp
        src/Viewer.cpp
        src/ifco_scene.cpp
        src/service_worker.cpp
        src/jacobian_controller.cpp
        src/soma_cerrt.cpp
        src/soma_concerrt.cpp
        src/workspace_samplers.cpp
        src/workspace_checkers.cpp
        src/collision_types.cpp
    )

    qt4_wrap_cpp(
        MOC_SRCS
        src/Viewer.h
        src/service_worker.h
        src/ifco_scene.h
        src/jacobian_controller.h
        OPTIONS
        -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
    )

    add_executable(
        tub_feasibility_check
        ${SRCS}
        ${MOC_SRCS}
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
