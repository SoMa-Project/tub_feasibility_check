#ifndef TABLETOP_SCENE_H
#define TABLETOP_SCENE_H

#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include "usecase_scene.h"
#include "process_table.h"

class TabletopScene : public UsecaseScene
{
public:
  TabletopScene(const std::string& scene_graph_file, const std::string& fixed_table_file,
                const std::string& kinematics_file);

  void createFixedTable(const rl::math::Transform& table_pose);
  void createTableFromEdges(const TableDescription& table_description);

private:
  std::size_t table_model_index_;
  SoVRMLGroup* table_vrml_group_;
};

#endif  // TABLETOP_SCENE_H
