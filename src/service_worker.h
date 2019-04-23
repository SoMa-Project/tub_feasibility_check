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
#include "tub_feasibility_check/CheckKinematics.h"
#include "tub_feasibility_check/CheckKinematicsTabletop.h"
#include "tub_feasibility_check/CerrtExample.h"
#include "tub_feasibility_check/VisualizeTrajectory.h"

#include "mainwindow.h"
#include "bounding_box.h"
#include "collision_specification.h"
#include "workspace_checkers.h"
#include "workspace_samplers.h"
#include "ifco_scene.h"
#include "tabletop_scene.h"

class ServiceWorker : public QObject
{
  Q_OBJECT

public:
  ServiceWorker(std::unique_ptr<IfcoScene> ifco_scene, std::unique_ptr<TabletopScene> tabletop_scene);

  bool checkKinematicsIfcoQuery(tub_feasibility_check::CheckKinematics::Request& req,
                            tub_feasibility_check::CheckKinematics::Response& res);

  bool checkKinematicsTabletopQuery(tub_feasibility_check::CheckKinematicsTabletop::Request& req,
                            tub_feasibility_check::CheckKinematicsTabletop::Response& res);

  bool visualizeTrajectoryQuery(tub_feasibility_check::VisualizeTrajectory::Request& req,
                                tub_feasibility_check::VisualizeTrajectory::Response& res);

  bool cerrtExampleQuery(tub_feasibility_check::CerrtExample::Request& req,
                         tub_feasibility_check::CerrtExample::Response& res);

  void reloadKinematics(std::string kinematics_file);

  void start(unsigned rate);

public slots:
  void spinOnce();
  void stop();

signals:
  void drawConfiguration(const rl::math::Vector& size);
  void drawBox(const rl::math::Vector& size, const rl::math::Transform& transform);
  void drawNamedFrame(const rl::math::Transform& transform, const std::string& name);
  void resetBoxes();
  void resetPoints();
  void resetLines();
  void toggleWorkFrames(bool on);

  void selectViewer(MainWindow::ViewerType type);

private:
  /* Visualize the position part of the goal manifold in the viewer. The manifold is specified by
   * the goal pose, and the minimum and maximum deviations from the goal pose in each coordinate.
   *
   * @param pose The goal pose.
   * @param zero_dimension_correction If exactly two of the manifold sizes are zero, set one of them to this number,
   * because viewer will not visualize a box with only one non-zero size.
   */
  void drawGoalManifold(rl::math::Transform pose, const boost::array<double, 3>& min_position_deltas,
                        const boost::array<double, 3>& max_position_deltas, double zero_dimension_correction = 0.01);

  std::unique_ptr<IfcoScene> ifco_scene;
  std::unique_ptr<TabletopScene> tabletop_scene;
  QTimer loop_timer;

  QMutex keep_running_mutex;
  bool keep_running = true;
};

#endif  // KINEMATICS_CHECK_H
