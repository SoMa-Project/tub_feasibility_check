#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <vector>
#include <rl/math/Vector.h>
#include <iostream>

namespace utilities
{
inline std::vector<rl::math::Real> eigenToStd(const rl::math::Vector& eigen_vector)
{
  return std::vector<rl::math::Real>(eigen_vector.data(), eigen_vector.data() + eigen_vector.size());
}

inline std::vector<rl::math::Real>concatanateEigneToStd(std::vector<rl::math::Vector> trajectory, const int DOF)
{
  int size = trajectory.size()*DOF;
  std::vector<rl::math::Real> new_trajectory(size,-1);

  for(auto i = 0; i < trajectory.size(); i++)
  {
    std::vector<rl::math::Real> pointOnTrajectory = eigenToStd(trajectory[i]);
    for (auto j = 0; j < DOF; j++)
    {
      new_trajectory[i*DOF+j] = pointOnTrajectory[j];
    }
  }

  return new_trajectory;
}

template <typename T>
rl::math::Vector stdToEigen(const std::vector<T>& std_vector)
{
  rl::math::Vector eigen_vector(std_vector.size());
  for (std::size_t i = 0; i < std_vector.size(); ++i)
    eigen_vector(i) = std_vector[i];
  return eigen_vector;
}

/* Needed for older compiler versions, that do not have a default hash for enum classes. */
struct EnumClassHash
{
  template <typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

}  // namespace utilities

#endif
