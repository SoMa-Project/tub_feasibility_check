#include "surface_grasp_pregrasp_manifold.h"

SurfaceGraspPregraspManifold::SurfaceGraspPregraspManifold(SurfaceGraspPregraspManifold::Description description)
  : description_(description),
    checker_(BoxPositionChecker(description.position_frame, description.min_position_deltas,
                                description.max_position_deltas),
             AroundTargetOrientationChecker(rl::math::Rotation(description.orientation),
                                            description.min_orientation_deltas, description.max_orientation_deltas))
  , sampler_(UniformPositionInAsymmetricBox(description.position_frame, description.min_position_deltas,
                                            description.max_position_deltas),
             DeltaXYZOrientation(description.orientation, description.min_orientation_deltas,
                                 description.max_orientation_deltas))
{
}

const WorkspaceChecker& SurfaceGraspPregraspManifold::checker()
{
  return checker_;
}

const WorkspaceSampler& SurfaceGraspPregraspManifold::sampler()
{
  return sampler_;
}

const SurfaceGraspPregraspManifold::Description& SurfaceGraspPregraspManifold::description()
{
  return description_;
}
