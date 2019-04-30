#ifndef TABLETOP_SCENE_H
#define TABLETOP_SCENE_H

#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include "usecase_scene.h"
#include "process_table.h"

class TabletopScene : public UsecaseScene
{
public:
  TabletopScene(const std::string& scene_graph_file, const std::string& kinematics_file,
                std::array<double, 3> fixed_table_dimensions);

  void createFixedTable(const rl::math::Transform& table_pose);
  void createTableFromEdges(const TableDescription& table_description);

  std::array<double, 3> getFixedTableDimensions() {
      return fixed_table_dimensions_;
  }

private:
  std::array<double, 3> fixed_table_dimensions_;
  const bool normal_points_downwards_ = true;

  std::size_t table_model_index_;
  SoVRMLGroup* table_vrml_group_;
};

#endif  // TABLETOP_SCENE_H
