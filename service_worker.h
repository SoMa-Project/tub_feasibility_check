#ifndef KINEMATICS_CHECK_H
#define KINEMATICS_CHECK_H

#include <QApplication>
#include <QMetaType>
#include <QThread>
#include <QTimer>
#include <QMutex>
#include <QObject>
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

class ServiceWorker : public QObject
{
  Q_OBJECT

public:
  ServiceWorker(std::unique_ptr<IfcoScene> ifco_scene) : QObject(nullptr), ifco_scene(std::move(ifco_scene))
  {
  }

  bool query(kinematics_check::CheckKinematics::Request& req, kinematics_check::CheckKinematics::Response& res);
  void start(unsigned rate);

public slots:
  void spinOnce();
  void stop();

private:
  std::unique_ptr<IfcoScene> ifco_scene;
  QTimer loop_timer;

  QMutex keep_running_mutex;
  bool keep_running = true;
};

#endif  // KINEMATICS_CHECK_H
