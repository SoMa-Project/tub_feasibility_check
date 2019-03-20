#ifndef TABLETOP_SCENE_H
#define TABLETOP_SCENE_H

#include "usecase_scene.h"

class TabletopScene : public UsecaseScene
{
public:
  TabletopScene(const std::string& scene_graph_file, const std::string& kinematics_file, bool mdl_format);

  void moveTable(const rl::math::Transform& table_pose);

private:
  std::size_t table_model_index_;
};

#endif // TABLETOP_SCENE_H
