#define BOOST_TEST_MODULE test_jacobian_controller
#include <QApplication>
#include <boost/test/included/unit_test.hpp>
#include <rl/plan/DistanceModel.h>
#include "../ifco_scene.h"
#include "../jacobian_controller.h"
#include "../MainWindow.h"

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

    controller = ifco_scene->makePlanner<JacobianController>();

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
  settings.number_of_particles = 30;
  settings.initial_std_error = Vector::Zero(ifco_scene->dof());
  settings.joints_std_error = Vector::Ones(ifco_scene->dof()) * 0.001;

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
    BOOST_REQUIRE(particles[i].config != particles[i+1].config);
}

BOOST_AUTO_TEST_SUITE_END()
