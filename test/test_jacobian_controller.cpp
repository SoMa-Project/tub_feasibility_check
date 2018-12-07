#define BOOST_TEST_MODULE test_jacobian_controller

#include <QApplication>
#include <boost/test/included/unit_test.hpp>
#include <rl/plan/DistanceModel.h>
#include <Eigen/Geometry>
#include "../ifco_scene.h"
#include "../jacobian_controller.h"
#include "../MainWindow.h"
#include "../workspace_constraints.h"

using namespace rl::math;
using namespace rl::plan;
using namespace boost::unit_test;

struct Fixture
{
  Fixture()
  {
    std::string default_root_dir = "/home/ilia/cmp_2/contact-motion-planning";
    std::string scene_graph_file = default_root_dir + "/soma/rlsg/wam-rbohand-ifco.convex.xml";
    std::string kinematics_file = default_root_dir + "/soma/rlkin/barrett-wam-ocado2.xml";

    ifco_scene = IfcoScene::load(scene_graph_file, kinematics_file);
    Transform t;
    ifco_scene->moveIfco(t.translate(Vector3(1000, 1000, 1000)));  // goodbye

    controller = ifco_scene->makePlanner<JacobianController>(0.017);

    settings.delta = 0.017;
    pose_in_front.translation() = Vector3(0.45, -0.4, 0.35);
    pose_in_front.linear() = Quaternion(0.1830127, 0.6830127, -0.6830127, 0.1830127).matrix();

    initial_configuration.resize(ifco_scene->dof());
    initial_configuration << 0.1, 0.1, 0, 2.3, 0, 0.5, 0;

    kinematics = rl::kin::Kinematics::create(kinematics_file);
    scene.load(scene_graph_file);
    model.kin = kinematics;
    model.model = scene.getModel(0);
    model.scene = &scene;
  }

  std::unique_ptr<IfcoScene> ifco_scene;
  std::shared_ptr<JacobianController> controller;
  JacobianController::Settings settings;
  Transform pose_in_front = Transform::Identity();
  Vector initial_configuration;

  // to check the solutions
  rl::kin::Kinematics* kinematics;
  rl::sg::bullet::Scene scene;
  rl::plan::DistanceModel model;

  // number of trials for sampling tests
  const unsigned trials = 100;
  // number of maximum sample attempts for sampling tests
  const unsigned maximum_sample_attempts = 10000;
};

BOOST_FIXTURE_TEST_SUITE(jacobian_controller_suite, Fixture)

BOOST_AUTO_TEST_CASE(multiple_particles_no_error)
{
  settings.number_of_particles = 30;
  settings.initial_std_error = Vector::Zero(ifco_scene->dof());
  settings.joints_std_error = Vector::Zero(ifco_scene->dof());

  auto result = controller->go(initial_configuration, pose_in_front, {}, settings);
  auto& particles = result.final_belief.getParticles();

  BOOST_CHECK(result);
  BOOST_CHECK(particles.size() == settings.number_of_particles);
  BOOST_CHECK(std::all_of(particles.begin(), particles.end(),
                          [particles](const Particle& p) { return p.config == particles.front().config; }));

  model.setPosition(result.final_belief.configMean());
  model.updateFrames();
  model.updateJacobian();
  model.updateJacobianInverse();
  auto ee_position = model.forwardPosition();
  BOOST_CHECK(ee_position.isApprox(pose_in_front, settings.delta));
}

BOOST_AUTO_TEST_CASE(no_initial_error_motion_error)
{
  settings.number_of_particles = 100;
  settings.initial_std_error = Vector::Zero(ifco_scene->dof());
  settings.joints_std_error = Vector::Ones(ifco_scene->dof()) * 0.01;

  auto result = controller->go(initial_configuration, pose_in_front, {}, settings);
  BOOST_CHECK(result);
  model.setPosition(result.final_belief.configMean());
  model.updateFrames();
  model.updateJacobian();
  model.updateJacobianInverse();
  auto ee_position = model.forwardPosition();
  BOOST_CHECK(ee_position.isApprox(pose_in_front, settings.delta));

  auto& particles = result.final_belief.getParticles();
  for (std::size_t i = 0; i < particles.size() - 1; ++i)
    BOOST_REQUIRE(particles[i].config != particles[i + 1].config);

  Vector travelled_conf_distance = Vector::Zero(ifco_scene->dof());
  for (std::size_t i = 1; i < result.mean_trajectory.size(); ++i)
    travelled_conf_distance += (result.mean_trajectory[i] - result.mean_trajectory[i - 1]).array().abs().matrix();

  Matrix theoretical_covariance =
      (travelled_conf_distance.array().abs() * settings.joints_std_error.array().square()).matrix().asDiagonal();

  auto theoretical_norm = theoretical_covariance.norm();
  auto real_norm = result.final_belief.configCovariance().norm();
  auto difference_norm = (theoretical_covariance - result.final_belief.configCovariance()).norm();

  std::cout << "Theoretical norm: " << theoretical_norm << ", real norm: " << real_norm
            << ", difference norm: " << difference_norm << "\n";

  BOOST_CHECK(theoretical_covariance.isApprox(result.final_belief.configCovariance()));
}

BOOST_AUTO_TEST_CASE(test_halfsphere_sample_without_rotation)
{
  const double radius = 0.5;

  auto center = Transform::Identity();
  center.translate(Vector3(0.45, -0.4, 0.35));

  Vector initial_configuration(7);
  initial_configuration << 0.1, 0.1, 0, 2.3, 0, 0.5, 0;

  auto halfSphere = WorkspaceConstraints::upperHalfSphere(center, radius);
  for (unsigned i = 0; i < trials; ++i)
  {
    auto sample = controller->sample(initial_configuration, {}, halfSphere, maximum_sample_attempts);
    BOOST_REQUIRE(sample);

    model.setPosition(*sample);
    model.updateFrames();
    auto sampled_point = center.inverse() * model.forwardPosition();
    BOOST_CHECK(sampled_point.translation().norm() <= radius);
    BOOST_CHECK(sampled_point.translation().z() >= 0);
  }
}

BOOST_AUTO_TEST_CASE(test_halfsphere_sample_with_rotation)
{
  const double radius = 0.5;

  auto center = Transform::Identity();
  center.rotate(Eigen::AngleAxisd(1.1, Vector3(0.57735027, 0.57735027, 0.57735027)));
  center.translate(Vector3(0.45, -0.4, 0.35));

  Vector initial_configuration(7);
  initial_configuration << 0.1, 0.1, 0, 2.3, 0, 0.5, 0;
  auto halfSphere = WorkspaceConstraints::upperHalfSphere(center, radius);
  for (unsigned i = 0; i < trials; ++i)
  {
    auto sample = controller->sample(initial_configuration, {}, halfSphere, maximum_sample_attempts);
    BOOST_REQUIRE(sample);

    model.setPosition(*sample);
    model.updateFrames();
    auto sampled_point = model.forwardPosition();
    sampled_point = center.inverse() * sampled_point;
    BOOST_CHECK(sampled_point.translation().norm() <= radius);
    BOOST_CHECK(sampled_point.translation().z() >= 0);
  }
}

BOOST_AUTO_TEST_SUITE_END()
