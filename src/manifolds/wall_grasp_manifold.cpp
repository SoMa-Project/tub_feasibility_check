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

  AlignDirectionOnPlaneTask align_task;
  align_task.surface_frame = description_.surface_frame;
  align_task.object_centroid = description_.object_centroid;
  align_task.initial_orientation = description_.orientation;
  align_task.frame_position = sampled_frame;
  align_task.direction_to_align = Eigen::Vector3d::UnitZ();
  align_task.cancel_x_out = false;

  auto rotation_towards_centroid = alignDirectionOnSurface(align_task);

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
