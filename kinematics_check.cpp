//
// Copyright (c) 2018, Can Erdogan & Arne Sievelring
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <QApplication>
#include <QMetaType>
#include <QThread>
#include <stdexcept>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/plan/VectorList.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "kinematics_check/CheckKinematics.h"

#include "MainWindow.h"

MainWindow* MainWindow::singleton = NULL;


class ROSThread : public QThread
{
public:
    void run(){

      //Set the update rate at which the interface receives motion commands
      ros::Rate loopRate = 20;

      while (ros::ok())
      {
        ros::spinOnce();
        //std::cout<<"spin"<<std::endl;

        loopRate.sleep();
      }
  }
};

bool query(kinematics_check::CheckKinematics::Request  &req,
         kinematics_check::CheckKinematics::Response &res)
{

  ROS_INFO("Recieving query");
  MainWindow* mw = MainWindow::instance();
  mw->start = boost::make_shared< rl::math::Vector >(req.joints.size());

  for(int i=0; i<req.joints.size(); i++)
  {
    (*mw->start)(i) = req.joints[i];
  }


  // Create a frame from the position/quaternion data
  mw->goalFrame = boost::make_shared< rl::math::Transform >();

  geometry_msgs::Pose goalPose = req.goalFrame;
  Eigen::Quaternion <double> q (goalPose.orientation.w, goalPose.orientation.x, goalPose.orientation.y, goalPose.orientation.z);
  mw->goalFrame->linear() = q.toRotationMatrix();
  mw->goalFrame->translation() = Eigen::Vector3d(goalPose.position.x, goalPose.position.y, goalPose.position.z);

  mw->desiredCollObj = req.collObject;

  ROS_INFO("Planning");

  mw->plan();

  ROS_INFO("Sending back response");

  res.success = mw->lastPlanningResult;

  rl::math::Vector lastConfig = mw->lastTrajectory[mw->lastTrajectory.size()-1];

  res.finalJoints.resize(req.joints.size());
  for(int i=0; i<req.joints.size(); i++)
  {
    res.finalJoints[i] = lastConfig(i);
  }
  return true;
}


int
main(int argc, char** argv)
{

	try
	{

    ros::init(argc, argv, "check_kinematics_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("check_kinematics", query);


		QApplication application(argc, argv);
		
		qRegisterMetaType< rl::math::Real >("rl::math::Real");
		qRegisterMetaType< rl::math::Transform >("rl::math::Transform");
		qRegisterMetaType< rl::math::Vector >("rl::math::Vector");
		qRegisterMetaType< rl::plan::VectorList >("rl::plan::VectorList");
		qRegisterMetaType< SbColor >("SbColor");
		
		QObject::connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));


    // Decide if the main window should be hidden based on the input parameter '--hide'
    if(argc > 1) {
      bool foundHide = false;
      for(int i = 1; i < argc; i++) {
        if(strcmp(argv[i], "--hide") == 0) {
          foundHide = true;
          break;
        }
      }
      if(foundHide)
        MainWindow::instance()->hide();
      else
        MainWindow::instance()->show();
    }
    else
      MainWindow::instance()->show();

    ROSThread rosThread;
    rosThread.start();

		return application.exec();
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return -1;
	}
}
