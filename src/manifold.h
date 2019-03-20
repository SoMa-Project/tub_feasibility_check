#ifndef MANIFOLD_H
#define MANIFOLD_H

#include <functional>
#include <rl/math/Transform.h>
#include <Inventor/nodes/SoNode.h>

class SoVRMLAppearance;
class SoVRMLTransform;

class Manifold
{
public:
  typedef std::function<double()> SampleRandom01;

  virtual ~Manifold();

  virtual bool contains(const rl::math::Transform& transform_to_check) const = 0;
  virtual rl::math::Transform generate(SampleRandom01 sample_random_01) const = 0;
  virtual SoNode* visualization() const = 0;
};

#endif  // MANIFOLD_H
