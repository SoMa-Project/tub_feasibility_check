cmake_minimum_required(VERSION 2.8.3)
project(kinematics_check)

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


if(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND ) # FALSE) #
        set(SRCS
		src/MainWindow.cpp
		src/kinematics_check.cpp
		src/Viewer.cpp
                src/ifco_scene.cpp
                src/service_worker.cpp
              src/jacobian_controller.cpp
              src/soma_cerrt.cpp
    src/workspace_samplers.cpp
    src/allowed_collisions.cpp)
	
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
		kinematics_check
                ${SRCS}
		${MOC_SRCS}
	)

        

	target_compile_definitions(
		kinematics_check
		PUBLIC
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
		${COIN_DEFINITIONS}
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
		${COIN_INCLUDE_DIRS}
		${ROBLIB_INCLUDE_DIRS}
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
		${COIN_LIBRARIES}
        )

add_dependencies(kinematics_check kinematics_check_generate_messages)

endif(QT_FOUND AND SOQT_FOUND AND BULLET_FOUND)
