#include <iostream>
#include <Eigen/Core>
#include <rl/math/Rotation.h>
#include <rl/math/Vector.h>
#include <boost/optional.hpp>

#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>

#include "old_manifold.h"

/* Convert a rotation to matrix to corresponding XYZ Euler angles. The ranges of resulting angles
 * are following: X (-pi, pi) Y (-pi/2, pi/2) Z (-pi, pi).
 */
std::array<double, 3> convertToXYZEuler(const Eigen::Matrix3d& rotation_matrix)
{
  std::array<double, 3> result;
  result[0] = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
  result[1] = std::atan2(-rotation_matrix(2, 0),
                         std::sqrt(std::pow(rotation_matrix(2, 1), 2) + std::pow(rotation_matrix(2, 2), 2)));
  result[2] = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

  return result;
}

OldManifold::OldManifold(Description description) : description_(description)
{
  appearance_ = new SoVRMLAppearance;
  auto material = new SoVRMLMaterial;
  material->diffuseColor.setValue(0.2f, 0.2f, 0.2f);
  material->transparency.setValue(0.75f);
  appearance_->material.setValue(material);
}

bool OldManifold::contains(const rl::math::Transform& transform_to_check) const
{
  Eigen::Vector3d position_difference = description_.frame.inverse() * transform_to_check.translation();

  std::cout << "Position difference: " << position_difference.transpose() << "\n";

  for (std::size_t i = 0; i < 3; ++i)
    if (position_difference(i) + position_epsilon_ < description_.min_position_deltas[i] ||
        position_difference(i) - position_epsilon_ > description_.max_position_deltas[i])
      return false;

  rl::math::Rotation orientation_difference = transform_to_check.linear() * description_.orientation.inverse();

  // Eigen eulerAngles is not used, because the range of first angle is (0, pi), which is
  // not good for us - we need a range symmetric around 0
  auto XYZ_euler_angles = convertToXYZEuler(orientation_difference);

  std::cout << "XYZ Euler rotation difference: ";
  for (auto& c : XYZ_euler_angles)
    std::cout << c << " ";
  std::cout << std::endl;

  // angle_epsilon_ needed because of the imprecision of rotation matrix calculations, i.e. when the sampled
  // rotation is 0 in one angle
  for (std::size_t i = 0; i < 3; i++)
    if (XYZ_euler_angles[i] + angle_epsilon_ < description_.min_orientation_deltas[i] ||
        XYZ_euler_angles[i] - angle_epsilon_ > description_.max_orientation_deltas[i])
      return false;

  return true;
}

rl::math::Transform OldManifold::generate(Manifold::SampleRandom01 sample_random_01) const
{
  rl::math::Transform sampled;
  for (int i = 0; i < 3; ++i)
    sampled.translation()(i) =
        description_.min_position_deltas[i] +
        (description_.max_position_deltas[i] - description_.min_position_deltas[i]) * sample_random_01();

  sampled.translation() = description_.frame * sampled.translation();

  std::array<double, 3> angles;
  for (unsigned i = 0; i < 3; ++i)
    angles[i] = description_.min_orientation_deltas[i] +
                (description_.max_orientation_deltas[i] - description_.min_orientation_deltas[i]) * sample_random_01();

  rl::math::Quaternion sampled_diff = rl::math::AngleAxis(angles[2], rl::math::Vector3::UnitZ()) *
                                      rl::math::AngleAxis(angles[1], rl::math::Vector3::UnitY()) *
                                      rl::math::AngleAxis(angles[0], rl::math::Vector3::UnitX());

  sampled.linear() = rl::math::Rotation(sampled_diff * description_.orientation);

  return sampled;
}

SoNode* OldManifold::visualization() const
{
  using namespace rl::math;

  Vector3 size;
  Vector3 center_correction;
  unsigned zero_count = 0;
  boost::optional<unsigned> zero_index;
  Transform pose(description_.frame);

  for (unsigned i = 0; i < 3; ++i)
  {
    center_correction(i) = description_.min_position_deltas[i] / 2 + description_.max_position_deltas[i] / 2;
    size(i) = description_.max_position_deltas[i] - description_.min_position_deltas[i];
    if (!size(i))
    {
      ++zero_count;
      zero_index = i;
    }
  }

  if (zero_count == 2)
  {
    assert(zero_index.is_initialized());
    size(*zero_index) = zero_dimension_correction_;
  }
  pose.translate(center_correction);

  auto vrml_transform = new SoVRMLTransform();
  const auto& translation = pose.translation();
  rl::math::Quaternion rotation(pose.rotation());
  vrml_transform->translation.setValue(translation(0), translation(1), translation(2));
  vrml_transform->rotation.setValue(rotation.x(), rotation.y(), rotation.z(), rotation.w());

  auto shape = new SoVRMLShape();
  shape->appearance = appearance_;

  auto box = new SoVRMLBox();
  box->size.setValue(size(0), size(1), size(2));

  shape->geometry = box;
  vrml_transform->addChild(shape);

  return vrml_transform;
}
