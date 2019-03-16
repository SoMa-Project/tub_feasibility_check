#ifndef JACOBIAN_CONTROLLER_H
#define JACOBIAN_CONTROLLER_H

#include <random>
#include <QObject>
#include <rl/kin/Kinematics.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/BeliefState.h>
#include <rl/plan/UniformSampler.h>
#include "Viewer.h"
#include "collision_specification.h"
#include "manifold.h"
#include "utilities.h"
#include <unordered_map>

class WorkspaceSampler;

class JacobianController : public QObject
{
  Q_OBJECT
public:
  /* Represents the result of moveSingleParticle. */
  struct SingleResult
  {
    /* Possible outcomes. */
    enum class Outcome
    {
      REACHED,
      TERMINATING_COLLISION,
      PROHIBITED_COLLISION,
      UNSENSORIZED_COLLISION,
      SINGULARITY,
      JOINT_LIMIT,
      STEPS_LIMIT,
      MISSING_REQUIRED_COLLISIONS,
      TERMINATED_OUTSIDE_GOAL_MANIFOLD
    };

    /* A structure storing additional outcome information. When Outcome is collision-related, the collisions
     * will be stored. When it is JOINT_LIMIT, the joint indices will be stored.
     */
    struct OutcomeInformation
    {
      std::vector<std::pair<std::string, std::string>> collisions;
      std::vector<unsigned> joint_indices;

      static OutcomeInformation CollisionInformation(std::vector<std::pair<std::string, std::string>> collisions);
      static OutcomeInformation JointNumbers(std::vector<unsigned> joint_indices);
    };

    /* The trajectory steps from start to termination. */
    std::vector<rl::math::Vector> trajectory;

    /* A map of outcomes.
     * It contains either only one positive outcome: REACHED or TERMINATING_COLLISION,
     * or one or more negative outcomes, that led to the termination of the planner. The outcomes
     * are mapped to additional outcome information.
     */
    std::unordered_map<Outcome, OutcomeInformation, utilities::EnumClassHash> outcomes;

    /* SingleResult converts to true when the termination was successful and
     * false otherwise.
     */
    operator bool() const;

    /* Clear outcomes and put one outcome with information there and return self. */
    SingleResult& setSingleOutcome(Outcome outcome, OutcomeInformation outcome_information = OutcomeInformation());

    /* Add one outcome with information in outcomes and return self. */
    SingleResult& addSingleOutcome(Outcome outcome, OutcomeInformation outcome_information = OutcomeInformation());
  };

  /* Represents the result of moveBelief. */
  struct BeliefResult
  {
    /* Stores the result of the first phase of moveBelief: no noise test.
     * If it is a failure, particle_results will not be initialized.
     */
    SingleResult no_noise_test_result;

    /* Stores the result of the second phase of moveBelief: the propagation of the belief.
     * Each particle has its own SingleResult.
     */
    boost::optional<std::vector<SingleResult>> particle_results;

    /* BeliefResult converts to true when no_noise_test_result is successful and every
     * particle's SingleResult is successful.
     */
    operator bool() const;
  };

  /* Particle count and noise settings for moveBelief. */
  struct MoveBeliefSettings
  {
    std::size_t number_of_particles;
    rl::math::Vector initial_std_error;
    rl::math::Vector joints_std_error;
  };

  /* Create a jacobian controller.
   *
   * @param kinematics The kinematics of the robot. Careful! Do not use the same kinematics object as in viewer!
   * @param bullet_scene The bullet scene. Will not be modified.
   * @param delta The step for simulation.
   * @param maximum_steps An upper limit for amount of steps executed during moveSingleParticle. Prevents infinite
   * cycles.
   * @param viewer A viewer that will be used to draw every step of moveSingleParticle.
   */
  JacobianController(std::shared_ptr<rl::kin::Kinematics> kinematics,
                     std::shared_ptr<rl::sg::bullet::Scene> bullet_scene, double delta, unsigned maximum_steps,
                     boost::optional<Viewer*> viewer = boost::none);

  /* Move from initial configuration to target pose using jacobian control and obeying collision constraints.
   * Note that a successful executing may not end in target pose, when there is a terminating collision.
   *
   * @param initial_configuration The start configuration.
   * @param target_pose The pose that the controller will try to achieve.
   * @param collision_types The specification of collision constraints and requirements.
   *
   * @return An object specifying the outcome and documenting every step of the trajectory taken.
   */
  SingleResult moveSingleParticle(const rl::math::Vector& initial_configuration, const rl::math::Transform& target_pose,
                                  const CollisionSpecification& collision_specification,
                                  boost::optional<const Manifold&> goal_manifold = boost::none);

  /* Create a belief in initial configuration and propagate it to the target pose using jacobian control and obeying
   * collision constraints. Done in two phases: first, a single particle is moved without noise to target pose.
   * If successful, the trajectory of the single particle is then repeated with multiple particles, sampling initial
   * and motion noise along the way.
   *
   * @param initial_configuration The start configuration. A belief will be created around it according to settings.
   * @param target_pose The pose that the controller will try to achieve.
   * @param collision_types The specification of collision constraints and requirements.
   * @param settings Specification of number of particles, and initial and motion error.
   *
   * @return An object storing results of both phases of execution.
   */
  BeliefResult moveBelief(const rl::math::Vector& initial_configuration, const rl::math::Transform& target_pose,
                          const CollisionSpecification& collision_types, MoveBeliefSettings settings);

private:
  typedef std::vector<std::pair<std::string, std::string>> CollisionPairs;
  struct CollisionConstraintsCheck
  {
    std::unordered_map<SingleResult::Outcome, SingleResult::OutcomeInformation, utilities::EnumClassHash> failures;
    std::vector<std::pair<std::string, std::string>> seen_terminating_collisions;
    bool success_termination = false;
  };

  std::vector<std::pair<std::string, std::string>>
  transformCollisionMapToNamePairs(const rl::sg::CollisionMap& collision_map) const;

  /* Check that collision_map does not violate collision constraints provided by collision_types. Also counts the
   * seen collisions using required_counter.
   *
   * Current logic is the following: a prohibited collision results in failure. A ignored collision never results in
   * failure, even if the robot part touching it is unsensorized. A terminating collision will terminate the execution.
   * If all required collisions were seen and there were no other failures, then it leads to success, otherwise to
   * failure.
   */
  // TODO remove "hidden" usage of required_counter, could be misleading.
  CollisionConstraintsCheck checkCollisionConstraints(const rl::sg::CollisionMap& collision_map,
                                                      const CollisionSpecification& collision_types,
                                                      RequiredCollisionCounter& required_counter);

  rl::math::Vector calculateQDot(const rl::math::Vector& configuration, const rl::math::Transform& goal_pose,
                                 double delta);
  void moveBelief(rl::plan::BeliefState& belief, const std::vector<rl::math::Real>& q_dots);

  std::string getPartName(const std::string& address) const;
  bool isSensorized(const std::string& part_name) const;

  std::shared_ptr<rl::kin::Kinematics> kinematics_;
  std::shared_ptr<rl::sg::bullet::Scene> bullet_scene_;
  rl::plan::NoisyModel noisy_model_;
  double delta_;
  unsigned maximum_steps_;

  std::mt19937 random_engine_;

  // TODO find a way to remove QT signals and slots so this class does not use QT but still is able to
  // visualize the execution in viewer.
signals:
  void drawConfiguration(const rl::math::Vector& config);
  void drawNamedFrame(const rl::math::Transform& transform, const std::string& name);
};

#endif  // JACOBIAN_CONTROLLER_H
