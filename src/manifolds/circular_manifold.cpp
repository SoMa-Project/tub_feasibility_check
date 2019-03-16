#include <rl/math/Rotation.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include "circular_manifold.h"

namespace SurfacePregraspManifolds
{
CircularManifold::CircularManifold(CircularManifold::Description description) : description_(description)
{
  appearance_ = new SoVRMLAppearance;
  auto material = new SoVRMLMaterial;
  material->diffuseColor.setValue(0.2f, 0.2f, 0.2f);
  material->transparency.setValue(0.75f);
  appearance_->material.setValue(material);
}

CircularManifold::~CircularManifold()
{
}

rl::math::Transform CircularManifold::generate(Manifold::SampleRandom01 sample_random_01) const
{
  double sampled_radius = std::sqrt(sample_random_01()) * description_.radius;
  double sampled_phi = sample_random_01() * M_PI * 2;
  double sampled_orientation_delta = (sample_random_01() - 0.5) * description_.orientation_delta;

  Eigen::Vector3d sampled_point_on_circle;
  sampled_point_on_circle(0) = sampled_radius * std::cos(sampled_phi);
  sampled_point_on_circle(1) = sampled_radius * std::sin(sampled_phi);
  sampled_point_on_circle(2) = 0;

  rl::math::Transform sampled_transform(description_.initial_frame);
  sampled_transform.translation() = description_.initial_frame * sampled_point_on_circle;
  sampled_transform.rotate(rl::math::AngleAxis(
      sampled_phi + sampled_orientation_delta - (description_.orient_outward ? 0 : M_PI), Eigen::Vector3d::UnitZ()));

  return sampled_transform;
}

bool CircularManifold::contains(const rl::math::Transform& transform_to_check) const
{
  Eigen::Affine3d difference_transform = description_.initial_frame.inverse() * transform_to_check;

  bool origin_in_plane = std::abs(difference_transform.translation()(2)) < z_axis_comparison_epsilon_;
  if (!origin_in_plane)
    return false;

  Eigen::AngleAxisd difference_in_orientation(difference_transform.linear());
  if (difference_in_orientation.axis()(2) < 0)
  {
    difference_in_orientation.axis() *= -1;
    difference_in_orientation.angle() *= -1;
  }

  bool rotated_around_z_only = difference_in_orientation.axis().isApprox(Eigen::Vector3d::UnitZ());
  if (!rotated_around_z_only)
    return false;

  double distance = difference_transform.translation().norm();
  double desired_rotation_angle =
      std::atan2(difference_transform.translation()(1) / distance, difference_transform.translation()(0) / distance);
  if (!description_.orient_outward)
    desired_rotation_angle -= M_PI;
  return std::abs(std::remainder(desired_rotation_angle - difference_in_orientation.angle(), 2 * M_PI)) <
         description_.orientation_delta + angle_comparison_epsilon_;
}

SoNode* CircularManifold::visualization() const
{
  using namespace rl::math;

  rl::math::Transform corrected_frame(description_.initial_frame);
  corrected_frame.rotate(rl::math::AngleAxis(M_PI / 2, Eigen::Vector3d::UnitX()));

  auto vrml_transform = new SoVRMLTransform();
  rl::math::Quaternion rotation(corrected_frame.rotation());
  vrml_transform->translation.setValue(corrected_frame.translation()(0), corrected_frame.translation()(1),
                                       corrected_frame.translation()(2));
  vrml_transform->rotation.setValue(rotation.x(), rotation.y(), rotation.z(), rotation.w());

  auto shape = new SoVRMLShape();
  shape->appearance = appearance_;

  auto cylinder = new SoVRMLCylinder();
  cylinder->radius.setValue(description_.radius);
  cylinder->height.setValue(visualization_cylinder_height_);

  shape->geometry = cylinder;
  vrml_transform->addChild(shape);
  return vrml_transform;
}

const Eigen::Affine3d& CircularManifold::initialFrame() const
{
  return description_.initial_frame;
}
}
