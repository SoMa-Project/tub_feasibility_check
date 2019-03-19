#include <rl/math/Rotation.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Inventor/VRMLnodes/SoVRMLCylinder.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include "circular_manifold.h"
#include "shared_functions.h"

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

  rl::math::Transform sampled_transform;
  sampled_transform.translation() = description_.position_frame * sampled_point_on_circle;
  sampled_transform.linear() = description_.orientation.matrix();

  auto rotation_towards_centroid = rotationTowardsCentroidOnSurfaceZ(
      description_.orientation, sampled_transform, description_.object_centroid, description_.surface_frame);
  rotation_towards_centroid.angle() += sampled_orientation_delta;
  if (description_.orient_outward)
    rotation_towards_centroid.angle() += M_PI;

  return sampled_transform.rotate(rotation_towards_centroid);
}

bool CircularManifold::contains(const rl::math::Transform& transform_to_check) const
{
  // not implemented due to time limit
  // currently unused in the combined service calls
  assert(false);
}

SoNode* CircularManifold::visualization() const
{
  using namespace rl::math;

  rl::math::Transform corrected_frame(description_.position_frame);
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
  return description_.position_frame;
}
}
