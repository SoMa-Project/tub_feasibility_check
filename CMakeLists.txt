cmake_minimum_required(VERSION 2.8.3)
project(kinematics_check)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations -fno-inline-functions")
set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};/opt/ros/indigo;/home/ilia/ec_grasp_planner/install)

add_definitions(-DEIGEN_DONT_ALIGN)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  geometry_msgs
  geometry_graph_msgs
  tf
  tf_conversions
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


add_service_files(
   FILES
   CheckKinematics.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_graph_msgs
 )



catkin_package(
  CATKIN_DEPENDS message_runtime
)

if(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND)
	set(
		HDRS		
		MainWindow.h
		Thread.h
		Viewer.h
	)
	
	set(
		SRCS
		MainWindow.cpp
		kinematics_check.cpp
		Thread.cpp
		Viewer.cpp
#		${rl_SOURCE_DIR}/robotics-library.rc
	)
	
	qt4_wrap_cpp(
		MOC_SRCS
		MainWindow.h
		Thread.h
		Viewer.h
		OPTIONS
		-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
	)
	
	add_executable(
		kinematics_check
		${HDRS}
		${SRCS}
		${MOC_SRCS}
	)
	
	target_compile_definitions(
		kinematics_check
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
	
endif(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND)
