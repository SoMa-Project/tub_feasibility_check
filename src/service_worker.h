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
#include <Inventor/nodes/SoNode.h>

#include "ros/ros.h"
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include "tub_feasibility_check/CheckKinematics.h"
#include "tub_feasibility_check/CheckKinematicsTabletop.h"
#include "tub_feasibility_check/CheckSurfaceGrasp.h"
#include "tub_feasibility_check/VisualizeTrajectory.h"

#include "mainwindow.h"
#include "bounding_box.h"
#include "collision_specification.h"
#include "ifco_scene.h"
#include "tabletop_scene.h"
#include "check_kinematics_parameters.h"
#include "jacobian_controller.h"

class ServiceWorker : public QObject
{
  Q_OBJECT

public:
  ServiceWorker(std::unique_ptr<IfcoScene> ifco_scene, std::unique_ptr<TabletopScene> tabletop_scene,
                double simulation_delta);

  bool checkKinematicsIfcoQuery(tub_feasibility_check::CheckKinematics::Request& req,
                                tub_feasibility_check::CheckKinematics::Response& res);

  bool checkKinematicsTabletopQuery(tub_feasibility_check::CheckKinematicsTabletop::Request& req,
                                    tub_feasibility_check::CheckKinematicsTabletop::Response& res);

  bool checkSurfaceGraspQuery(tub_feasibility_check::CheckSurfaceGrasp::Request& req,
                              tub_feasibility_check::CheckSurfaceGrasp::Response& res);

  bool visualizeTrajectoryQuery(tub_feasibility_check::VisualizeTrajectory::Request& req,
                                tub_feasibility_check::VisualizeTrajectory::Response& res);

  void start(unsigned rate);

public slots:
  void spinOnce();
  void stop();

signals:
  void drawConfiguration(const rl::math::Vector& size);
  void drawNode(SoNode* node);
  void drawBox(const rl::math::Vector& size, const rl::math::Transform& transform);
  void drawNamedFrame(const rl::math::Transform& transform, const std::string& name);
  void resetBoxes();
  void resetPoints();
  void resetLines();
  void resetFrames();
  void toggleWorkFrames(bool on);

  void selectViewer(MainWindow::ViewerType type);

private:
  struct SurfaceGraspResult
  {
    JacobianController::SingleResult pregrasp_result;
    boost::optional<JacobianController::SingleResult> go_down_result;

    operator bool() const;
    std::vector<rl::math::Vector> combinedTrajectory() const;
  };

  void clearViewerScene();

  SurfaceGraspResult trySurfaceGrasp(JacobianController& controller, const SharedParameters& shared_parameters,
                                     const CheckSurfaceGraspParameters& specific_parameters,
                                     const Eigen::Affine3d& pregrasp_goal, const Eigen::Affine3d& go_down_goal);

  std::unique_ptr<IfcoScene> ifco_scene;
  std::unique_ptr<TabletopScene> tabletop_scene;
  double delta_;
  double maximum_steps_;
  QTimer loop_timer;

  QMutex keep_running_mutex;
  bool keep_running = true;
};

#endif  // KINEMATICS_CHECK_H
