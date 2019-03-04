#include "tabletop_scene.h"

TabletopScene::TabletopScene(const std::string& scene_graph_file, const std::string& kinematics_file)
  : UsecaseScene(scene_graph_file, kinematics_file)
{
  auto index = findModelIndexByName("tabletop");
  if (!index)
  {
    std::stringstream ss;
    ss << "Tabletop scene: no tabletop found in " << scene_graph_file;
    throw std::runtime_error(ss.str());
  }

  table_model_index_ = *index;
}

void TabletopScene::moveTable(const rl::math::Transform& table_pose)
{
  auto findAndMoveIfco = [this, table_pose](rl::sg::Scene& scene) {
    scene.getModel(table_model_index_)->getBody(0)->setFrame(table_pose);
  };

  findAndMoveIfco(*bullet_scene_);
  emit applyFunctionToScene(findAndMoveIfco);
}
