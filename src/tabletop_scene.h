#ifndef TABLETOP_SCENE_H
#define TABLETOP_SCENE_H

#include "usecase_scene.h"
#include "process_table.h"

class TabletopScene : public UsecaseScene
{
public:
  TabletopScene(const std::string& scene_graph_file, const std::string& kinematics_file);

  void moveTable(const rl::math::Transform& table_pose);
  void createTable(const TableDescription& table_description);

private:
  std::size_t table_model_index_;
};

#endif  // TABLETOP_SCENE_H
