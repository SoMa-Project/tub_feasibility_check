#include <rl/math/Rotation.h>
#include <rl/math/Vector.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>

#include "wall_grasp_manifold.h"

WallGraspManifold::WallGraspManifold(WallGraspManifold::Description description) : description_(description)
{
  appearance_ = new SoVRMLAppearance;
  auto material = new SoVRMLMaterial;
  material->diffuseColor.setValue(0.2f, 0.2f, 0.2f);
  material->transparency.setValue(0.75f);
  appearance_->material.setValue(material);
}

bool WallGraspManifold::contains(const rl::math::Transform& transform_to_check) const
{
  // TODO implement this if you need to use it
  assert(false);

  Eigen::Vector3d diff = description_.initial_frame.inverse() * transform_to_check.translation();

  if (!diff.normalized().isApprox(Eigen::Vector3d::UnitY()) || !diff.normalized().isApprox(-Eigen::Vector3d::UnitY()))
    return false;

  if (diff.norm() > description_.width / 2 + position_comparison_epsilon_)
    return false;

  Eigen::Vector3d direction_towards_centroid =
      transform_to_check.inverse() * description_.object_centroid;
  direction_towards_centroid(2) = 0;
  direction_towards_centroid.normalize();
  return direction_towards_centroid.isApprox(Eigen::Vector3d::UnitX());
}

rl::math::Transform WallGraspManifold::generate(Manifold::SampleRandom01 sample_random_01) const
{
  Eigen::Vector3d sampled_translation = (sample_random_01() - 0.5) * description_.width * Eigen::Vector3d::UnitY();

  rl::math::Transform sampled_frame = description_.initial_frame;
  sampled_frame.translate(sampled_translation);

  Eigen::Vector3d towards_centroid_in_surface_frame =
      description_.surface_frame.rotation().inverse() *
      (description_.object_centroid - sampled_frame.translation());
  Eigen::Vector3d rotation_vector_in_sampled_frame =
      sampled_frame.linear().inverse() * description_.surface_frame.linear() * rl::math::Vector3::UnitZ();

  double rotation = M_PI + std::atan2(towards_centroid_in_surface_frame.y(), towards_centroid_in_surface_frame.x());

  return sampled_frame.rotate(rl::math::AngleAxis(rotation, rotation_vector_in_sampled_frame));
}

SoNode* WallGraspManifold::visualization() const
{
  auto vrml_transform = new SoVRMLTransform;
  const auto& translation = description_.initial_frame.translation();
  rl::math::Quaternion rotation(description_.initial_frame.rotation());
  vrml_transform->translation.setValue(translation(0), translation(1), translation(2));
  vrml_transform->rotation.setValue(rotation.x(), rotation.y(), rotation.z(), rotation.w());

  auto shape = new SoVRMLShape;
  shape->appearance = appearance_;

  auto box = new SoVRMLBox;
  box->size.setValue(visualization_box_zero_correction_, description_.width, visualization_box_zero_correction_);

  shape->geometry = box;
  vrml_transform->addChild(shape);

  return vrml_transform;
}

const Eigen::Affine3d& WallGraspManifold::initialFrame() const
{
  return description_.initial_frame;
}
