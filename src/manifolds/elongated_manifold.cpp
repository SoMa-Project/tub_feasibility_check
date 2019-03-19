#include <rl/math/Vector.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>

#include "elongated_manifold.h"

namespace SurfacePregraspManifolds
{
ElongatedManifold::ElongatedManifold(ElongatedManifold::Description description) : description_(description)
{
  appearance_ = new SoVRMLAppearance;
  auto material = new SoVRMLMaterial;
  material->diffuseColor.setValue(0.2f, 0.2f, 0.2f);
  material->transparency.setValue(0.75f);
  appearance_->material.setValue(material);
}

rl::math::Transform ElongatedManifold::generate(SampleRandom01 sample_random_01) const
{
  Eigen::Vector3d sampled_position_in_plane = Eigen::Vector3d::Zero();
  sampled_position_in_plane(0) = description_.stripe_width * (sample_random_01() - 0.5);
  sampled_position_in_plane(1) = description_.stripe_height * (sample_random_01() - 0.5);
  sampled_position_in_plane(1) += std::copysign(description_.stripe_offset, sampled_position_in_plane(1));

  double orientation = std::copysign(M_PI / 2, (description_.orient_outward ? 1 : -1) * sampled_position_in_plane(1));
  double sampled_orientation_delta = (sample_random_01() - 0.5) * description_.orientation_delta;

  rl::math::Transform sampled_transform;
  sampled_transform.translation() = description_.position_frame * sampled_position_in_plane;
  sampled_transform.rotate(Eigen::AngleAxisd(orientation + sampled_orientation_delta, Eigen::Vector3d::UnitZ()));
  return sampled_transform;
}

bool ElongatedManifold::contains(const rl::math::Transform& transform_to_check) const
{
  // not implemented due to time limit
  // currently unused in the combined service calls
  assert(false);
}

SoNode* ElongatedManifold::visualization() const
{
  auto group = new SoVRMLGroup;

  auto makeBox = [this](const Eigen::Vector3d& size, const Eigen::Affine3d& transform) {
    auto vrml_transform = new SoVRMLTransform();
    rl::math::Vector translation = transform.translation();
    rl::math::Quaternion rotation(transform.rotation());
    vrml_transform->translation.setValue(translation(0), translation(1), translation(2));
    vrml_transform->rotation.setValue(rotation.x(), rotation.y(), rotation.z(), rotation.w());

    auto shape = new SoVRMLShape();
    shape->appearance = appearance_;

    auto box = new SoVRMLBox();
    box->size.setValue(size(0), size(1), size(2));

    shape->geometry = box;
    vrml_transform->addChild(shape);

    return vrml_transform;
  };

  Eigen::Affine3d stripe_center = description_.position_frame;
  Eigen::Vector3d size(description_.stripe_width, description_.stripe_height, visualization_box_height_);
  stripe_center.translate(Eigen::Vector3d::UnitY() * (description_.stripe_offset + description_.stripe_height / 2));
  group->addChild(makeBox(size, stripe_center));

  stripe_center.translate(-Eigen::Vector3d::UnitY() * (2 * description_.stripe_offset + description_.stripe_height));
  group->addChild(makeBox(size, stripe_center));

  return group;
}

const Eigen::Affine3d& ElongatedManifold::initialFrame() const
{
  return description_.position_frame;
}
}
