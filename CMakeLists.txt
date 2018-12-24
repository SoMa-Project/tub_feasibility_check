cmake_minimum_required(VERSION 2.8.3)
project(kinematics_check)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations -Wno-zero-as-null-pointer-constant -fno-inline-functions")
set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};/opt/ros/indigo)

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
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../../Modules/")


find_package(Boost REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})

find_package(Bullet REQUIRED)
find_package(SoQt REQUIRED)
find_package(LibXml2 REQUIRED)

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


if(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND)
        set(
                HDRS
                MainWindow.h
                Viewer.h
                ifco_scene.h
                service_worker.h
                jacobian_controller.h
                utilities.h
				allowed_collisions.h
				workspace_samplers.h
				SomaCerrt.h
        )

        set(
		SRCS
		MainWindow.cpp
		kinematics_check.cpp
		Viewer.cpp
                ifco_scene.cpp
                service_worker.cpp
                jacobian_controller.cpp
#		${rl_SOURCE_DIR}/robotics-library.rc
				workspace_samplers.cpp SomaCerrt.cpp)
	
	qt4_wrap_cpp(
		MOC_SRCS
		Viewer.h
                service_worker.h
                ifco_scene.h
                jacobian_controller.h
		OPTIONS
		-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
	)
	
	add_executable(
		kinematics_check
                ${HDRS}
                ${SRCS}
		${MOC_SRCS}
	)

        qt4_wrap_cpp(
            TEST_MOC_SRCS
            ifco_scene.h
            jacobian_controller.h
        )
        add_executable(test_jacobian_controller
            ifco_scene.cpp
            jacobian_controller.cpp
            workspace_samplers.cpp
            test/test_jacobian_controller.cpp
            ${TEST_MOC_SRCS})
        add_test(NAME test_jacobian_controller COMMAND test_jacobian_controller)

	target_compile_definitions(
		kinematics_check
		PUBLIC
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
	)

        target_compile_definitions(
                test_jacobian_controller
                PUBLIC
                ${QT_DEFINITIONS}
                ${SOQT_DEFINITIONS}
        )

	target_include_directories(
		kinematics_check
		PUBLIC
		${Boost_INCLUDE_DIR}
		${catkin_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		${QT_INCLUDES}
		${SOQT_INCLUDE_DIRS}
		${BULLET_INCLUDE_DIRS}
		${LIBXML2_INCLUDE_DIRS}
	)

        target_include_directories(
                test_jacobian_controller
                PUBLIC
                ${Boost_INCLUDE_DIR}
                ${catkin_INCLUDE_DIRS}
                ${OPENGL_INCLUDE_DIR}
                ${QT_INCLUDES}
                ${SOQT_INCLUDE_DIRS}
                ${BULLET_INCLUDE_DIRS}
                ${LIBXML2_INCLUDE_DIRS}
        )

	target_link_libraries(
                test_jacobian_controller
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
	)

        target_link_libraries(
                kinematics_check
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
        )

add_dependencies(kinematics_check kinematics_check_generate_messages)

endif(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND)
