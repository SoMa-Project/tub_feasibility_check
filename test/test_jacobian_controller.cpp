#include <gtest/gtest.h>
#include <QApplication>
#include <rl/plan/DistanceModel.h>
#include <Eigen/Geometry>
#include "../src/ifco_scene.h"
#include "../src/jacobian_controller.h"
#include "../src/MainWindow.h"
#include "../src/workspace_samplers.h"

using namespace rl::math;
using namespace rl::plan;

class JacobianControllerBeliefsTest : public ::testing::Test
{
protected:
  JacobianControllerBeliefsTest()
  {
    std::string default_root_dir = "/home/ilia/cmp_2/contact-motion-planning";
    std::string scene_graph_file = default_root_dir + "/soma/rlsg/wam-rbohand-ifco.convex.xml";
    std::string kinematics_file = default_root_dir + "/soma/rlkin/barrett-wam-ocado2.xml";

    ifco_scene = IfcoScene::load(scene_graph_file, kinematics_file);
    Transform t;
    ifco_scene->moveIfco(t.translate(Vector3(1000, 1000, 1000)));  // goodbye

    controller = std::make_shared<JacobianController>(ifco_scene->getKinematics(), ifco_scene->getBulletScene(),
                                                      0.017, 10000);

    pose_in_front.translation() = Vector3(0.45, -0.4, 0.35);
    pose_in_front.linear() = Quaternion(0.1830127, 0.6830127, -0.6830127, 0.1830127).matrix();

    initial_configuration.resize(ifco_scene->dof());
    initial_configuration << 0.1, 0.1, 0, 2.3, 0, 0.5, 0;

    kinematics = rl::kin::Kinematics::create(kinematics_file);
    scene.load(scene_graph_file);
    model.kin = kinematics;
    model.model = scene.getModel(0);
    model.scene = &scene;

    allow_all_poses.reset(
        new WorkspaceChecker([](const Vector3&) { return true; }, [](const Rotation&) { return true; }));
  }

  std::unique_ptr<IfcoScene> ifco_scene;
  std::shared_ptr<JacobianController> controller;
  JacobianController::MoveBeliefSettings settings;
  Transform pose_in_front = Transform::Identity();
  Vector initial_configuration;

  AllowAllCollisions allow_all_collisions;
  std::unique_ptr<WorkspaceChecker> allow_all_poses;

  // to check the solutions
  rl::kin::Kinematics* kinematics;
  rl::sg::bullet::Scene scene;
  rl::plan::DistanceModel model;

  // number of trials for sampling tests
  const unsigned trials = 40;
  // number of maximum sample attempts for sampling tests
  const unsigned maximum_sample_attempts = 40;

  const double delta = 0.017;
};

TEST_F(JacobianControllerBeliefsTest, multiple_particles_no_error)
{
  settings.number_of_particles = 30;
  settings.joints_std_error = Vector::Zero(ifco_scene->dof());

  std::vector<rl::plan::Particle> initial_configs(settings.number_of_particles, Particle(initial_configuration));
  rl::plan::BeliefState initial_belief(initial_configs, &model);
  auto result = controller->moveBelief(initial_belief, pose_in_front, allow_all_collisions, *allow_all_poses, settings);
  auto belief = result.belief(model);
  ASSERT_TRUE(belief.is_initialized());
  auto& particles = belief->getParticles();

  ASSERT_TRUE(result);
  ASSERT_TRUE(result.particle_results.is_initialized());
  ASSERT_TRUE(result.particle_results->size() == settings.number_of_particles);
  ASSERT_TRUE(std::all_of(particles.begin(), particles.end(),
                          [particles](const Particle& p) { return p.config == particles.front().config; }));

  model.setPosition(belief->configMean());
  model.updateFrames();
  model.updateJacobian();
  model.updateJacobianInverse();
  auto ee_position = model.forwardPosition();
  ASSERT_TRUE(ee_position.isApprox(pose_in_front, delta));
}

TEST_F(JacobianControllerBeliefsTest, no_initial_error_motion_error)
{
  settings.number_of_particles = 100;
  settings.joints_std_error = Vector::Ones(ifco_scene->dof()) * 0.01;

  std::vector<rl::plan::Particle> initial_configs(settings.number_of_particles, Particle(initial_configuration));
  rl::plan::BeliefState initial_belief(initial_configs, &model);
  auto result = controller->moveBelief(initial_belief, pose_in_front, allow_all_collisions, *allow_all_poses, settings);
  auto belief = result.belief(model);
  auto& particles = belief->getParticles();

  ASSERT_TRUE(result);
  model.setPosition(belief->configMean());
  model.updateFrames();
  model.updateJacobian();
  model.updateJacobianInverse();
  auto ee_position = model.forwardPosition();
  ASSERT_TRUE(ee_position.isApprox(pose_in_front, delta));

  for (std::size_t i = 0; i < particles.size() - 1; ++i)
    ASSERT_TRUE(particles[i].config != particles[i + 1].config);

  Vector travelled_conf_distance = Vector::Zero(ifco_scene->dof());
  auto& mean_trajectory = result.no_noise_test_result.trajectory;
  for (std::size_t i = 1; i < mean_trajectory.size(); ++i)
    travelled_conf_distance += (mean_trajectory[i] - mean_trajectory[i - 1]).array().abs().matrix();

  Matrix theoretical_covariance =
      (travelled_conf_distance.array().abs() * settings.joints_std_error.array().square()).matrix().asDiagonal();

  auto theoretical_norm = theoretical_covariance.norm();
  auto real_norm = belief->configCovariance().norm();
  auto difference_norm = (theoretical_covariance - belief->configCovariance()).norm();

  std::cout << "Theoretical norm: " << theoretical_norm << ", real norm: " << real_norm
            << ", difference norm: " << difference_norm << "\n";

  // TODO a statistical test
  ASSERT_TRUE(theoretical_covariance.isApprox(belief->configCovariance()));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
