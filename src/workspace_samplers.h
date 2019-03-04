#ifndef _WORKSPACE_CONSTRAINTS_H_
#define _WORKSPACE_CONSTRAINTS_H_

#include <random>
#include <cmath>
#include <rl/math/Vector.h>
#include <rl/math/Transform.h>
#include <boost/optional.hpp>
#include "collision_specification.h"
#include "jacobian_controller.h"

// TODO in dire need of rework. Have not figured out a way to write it cleanly so far.

class UniformPositionInAsymmetricBox
{
public:
  UniformPositionInAsymmetricBox(rl::math::Transform box_pose, boost::array<double, 3> min_deltas,
                                 boost::array<double, 3> max_deltas);

  rl::math::Vector3 operator()(std::function<double()> sample_random_01);

private:
  rl::math::Transform box_pose_;
  boost::array<double, 3> min_deltas_;
  boost::array<double, 3> max_deltas_;
};

class DeltaXYZOrientation
{
public:
  DeltaXYZOrientation(rl::math::Quaternion initial_orientation, boost::array<double, 3> min_XYZ_deltas,
                      boost::array<double, 3> max_XYZ_deltas);

  rl::math::Quaternion operator()(std::function<double()> sample_random_01);
private:
  rl::math::Quaternion initial_orientation_;
  boost::array<double, 3> min_XYZ_deltas_;
  boost::array<double, 3> max_XYZ_deltas_;
};

rl::math::Quaternion
uniformOrientation(std::function<double()> sample_random_01);

class WorkspaceSampler
{
public:
  typedef std::function<double()> SampleRandom01;
  typedef std::function<rl::math::Vector3(SampleRandom01)> SamplePosition;
  typedef std::function<rl::math::Quaternion(SampleRandom01)> SampleOrientation;

  WorkspaceSampler(SamplePosition sample_position, SampleOrientation sample_orientation);

  rl::math::Transform generate(SampleRandom01 sample_random_01);

private:
  SamplePosition sample_position_;
  SampleOrientation sample_orientation_;
};

#endif
