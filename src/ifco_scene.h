#ifndef IFCO_SCENE_H
#define IFCO_SCENE_H

#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/kin/Kinematics.h>
#include <rl/plan/DistanceModel.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/sg/so/Scene.h>
#include <string>
#include <memory>
#include <unordered_map>
#include <QThread>
#include <QMetaType>

#include "Viewer.h"
#include "utilities.h"
#include "bounding_box.h"
#include "usecase_scene.h"

class IfcoScene : public UsecaseScene
{
public:
  IfcoScene(const std::string& scene_graph_file_, const std::string& kinematics_file_);

  void moveIfco(const rl::math::Transform& ifco_pose);

private:
  std::size_t ifco_model_index_;
};

#endif  // IFCO_SCENE_H
