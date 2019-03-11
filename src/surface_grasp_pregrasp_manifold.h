#ifndef SURFACE_GRASP_PREGRASP_MANIFOLD_H
#define SURFACE_GRASP_PREGRASP_MANIFOLD_H

#include <Eigen/Geometry>
#include <boost/array.hpp>
#include "workspace_checkers.h"
#include "workspace_samplers.h"

class SurfaceGraspPregraspManifold
{
public:
  struct Description
  {
    Eigen::Affine3d initial_frame;
    double radius;
  };

  SurfaceGraspPregraspManifold(Description description);

  const WorkspaceChecker& checker() const;
  const WorkspaceSampler& sampler() const;
  const Description& description() const;

private:
  struct ManifoldSampler final : public WorkspaceSampler
  {
    ManifoldSampler(Description description) : description_(description)
    {
    }

    rl::math::Transform generate(SampleRandom01 sample_random_01) const override;

    Description description_;
  };

  struct ManifoldChecker final : public WorkspaceChecker
  {
    ManifoldChecker(Description description) : description_(description)
    {
    }

    bool contains(const rl::math::Transform& transform_to_check) const override;

    Description description_;
  };

  Description description_;
  std::shared_ptr<WorkspaceChecker> checker_;
  std::shared_ptr<WorkspaceSampler> sampler_;
};

#endif  // SURFACE_GRASP_PREGRASP_MANIFOLD_H
