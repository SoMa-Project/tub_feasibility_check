#include <rl/math/Rotation.h>
#include <rl/math/Vector.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>

#include "wall_grasp_manifold.h"
#include "shared_functions.h"

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
}

rl::math::Transform WallGraspManifold::generate(Manifold::SampleRandom01 sample_random_01) const
{
  Eigen::Vector3d sampled_translation = (sample_random_01() - 0.5) * description_.width * Eigen::Vector3d::UnitX();

  rl::math::Transform sampled_frame = description_.position_frame;
  sampled_frame.translate(sampled_translation);
  sampled_frame.linear() = description_.orientation.matrix();

  auto rotation_towards_centroid = rotationTowardsCentroidOnSurfaceZ(
      description_.orientation, sampled_frame, description_.object_centroid, description_.surface_frame);
  if (description_.orient_outward)
    rotation_towards_centroid.angle() += M_PI;

  return sampled_frame.rotate(rotation_towards_centroid);
}

SoNode* WallGraspManifold::visualization() const
{
  auto vrml_transform = new SoVRMLTransform;
  const auto& translation = description_.position_frame.translation();
  rl::math::Quaternion rotation(description_.position_frame.rotation());
  vrml_transform->translation.setValue(translation(0), translation(1), translation(2));
  vrml_transform->rotation.setValue(rotation.x(), rotation.y(), rotation.z(), rotation.w());

  auto shape = new SoVRMLShape;
  shape->appearance = appearance_;

  auto box = new SoVRMLBox;
  box->size.setValue(description_.width, visualization_box_zero_correction_, visualization_box_zero_correction_);

  shape->geometry = box;
  vrml_transform->addChild(shape);

  return vrml_transform;
}

const Eigen::Affine3d& WallGraspManifold::initialFrame() const
{
  return description_.position_frame;
}
