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
#include "tub_feasibility_check/CerrtExample.h"
#include "tub_feasibility_check/VisualizeTrajectory.h"

#include "MainWindow.h"
#include "ifco_scene.h"

class ServiceWorker : public QObject
{
  Q_OBJECT

public:
  ServiceWorker(std::unique_ptr<IfcoScene> ifco_scene);

  bool checkKinematicsQuery(tub_feasibility_check::CheckKinematics::Request& req,
                            tub_feasibility_check::CheckKinematics::Response& res);

  bool cerrtExampleQuery(tub_feasibility_check::CerrtExample::Request& req,
                         tub_feasibility_check::CerrtExample::Response& res);

  bool visualizeTrajectoryQuery(tub_feasibility_check::VisualizeTrajectory::Request& req,
                                tub_feasibility_check::VisualizeTrajectory::Response& res);

  void start(unsigned rate);

public slots:
  void spinOnce();
  void stop();

signals:
  void drawConfiguration(const rl::math::Vector& config);
  void drawBox(const rl::math::Vector& size, const rl::math::Transform& transform);
  void drawWork(const rl::math::Transform& transform);
  void resetBoxes();
  void resetPoints();
  void resetLines();
  void toggleWorkFrames(bool on);

private:
  std::string getBoxName(std::size_t box_id) const;
  std::string getBoxShapeName(std::size_t box_id) const;
  std::size_t getBoxId(const std::string& box_name) const;

  /* Visualize the position part of the goal manifold in the viewer. The manifold is specified by
   * the goal pose, and the minimum and maximum deviations from the goal pose in each coordinate.
   *
   * @param pose The goal pose.
   * @param zero_dimension_correction If exactly two of the manifold sizes are zero, set one of them to this number,
   * because viewer will not visualize a box with only one non-zero size.
   */
  void drawGoalManifold(rl::math::Transform pose, const boost::array<double, 3>& min_position_deltas,
                        const boost::array<double, 3>& max_position_deltas, double zero_dimension_correction = 0.01);

  bool checkParameters(const tub_feasibility_check::CheckKinematics::Request& req);

  std::unique_ptr<IfcoScene> ifco_scene;
  QTimer loop_timer;

  QMutex keep_running_mutex;
  bool keep_running = true;

  std::array<double, 3> min_allowed_XYZ_angles{{ -M_PI, -M_PI / 2, -M_PI }};
  std::array<double, 3> max_allowed_XYZ_angles{{ M_PI, M_PI / 2, M_PI }};
};

#endif  // KINEMATICS_CHECK_H
