#ifndef KINEMATICS_CHECK_H
#define KINEMATICS_CHECK_H

#include <QApplication>
#include <QMetaType>
#include <QThread>
#include <stdexcept>
#include <random>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/math/Rotation.h>
#include <rl/plan/VectorList.h>

#include "ros/ros.h"
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include "kinematics_check/CheckKinematics.h"

#include "MainWindow.h"
#include "ifco_scene.h"

class ServiceThread : public QThread
{
public:
  ServiceThread(std::unique_ptr<IfcoScene> ifco_scene, ros::Rate loop_rate)
    : ifco_scene(std::move(ifco_scene)), loop_rate(loop_rate)
  {
  }

  void run();
  bool query(kinematics_check::CheckKinematics::Request& req, kinematics_check::CheckKinematics::Response& res);

private:
  std::unique_ptr<IfcoScene> ifco_scene;
  ros::Rate loop_rate;
};

#endif // KINEMATICS_CHECK_H
