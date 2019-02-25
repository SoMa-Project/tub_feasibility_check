#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include "tabletop_scene.h"

TabletopScene::TabletopScene(const std::string& scene_graph_file, const std::string& fixed_table_file,
                             const std::string& kinematics_file)
  : UsecaseScene(scene_graph_file, kinematics_file)
{
  SoInput in;
  if (!in.openFile(fixed_table_file.c_str()))
    throw std::runtime_error("Error reading fixed table");

  table_vrml_group_ = SoDB::readAllVRML(&in);

  auto model_index = findModelIndexByName("tabletop");
  if (!model_index)
    throw std::runtime_error("Failed to find tabletop model");

  table_model_index_ = *model_index;
}

void TabletopScene::createFixedTable(const rl::math::Transform& table_pose)
{
  auto findAndMoveIfco = [this, table_pose](rl::sg::Scene& scene) {
    auto tabletop_model = scene.getModel(table_model_index_);
    for (std::size_t i = 0; i < tabletop_model->getNumBodies(); ++i)
      tabletop_model->remove(tabletop_model->getBody(i));

    auto sg_body = tabletop_model->create();
    auto sg_shape = sg_body->create(dynamic_cast<SoVRMLShape*>(table_vrml_group_->getChild(0)));
    sg_shape->setName("fixed_tabletop");
    sg_shape->setTransform(table_pose);
  };

  findAndMoveIfco(*bullet_scene_);
  emit applyFunctionToScene(findAndMoveIfco);
}

void TabletopScene::createTableFromEdges(const TableDescription& table_description)
{
  std::vector<SbVec3f> points(table_description.points.size() * 2);
  std::vector<int32_t> indices(table_description.points.size() * 8 + 4);

  for (std::size_t i = 0; i < table_description.points.size(); ++i)
  {
    points[i].setValue(table_description.points[i].x(), table_description.points[i].y(),
                       table_description.points[i].z());
    Eigen::Vector3d downward_point = table_description.points[i] - table_description.normal.normalized() * TableHeight;
    points[table_description.points.size() + i].setValue(downward_point.x(), downward_point.y(), downward_point.z());
  }

  // upper face
  for (std::size_t i = 0; i < table_description.points.size(); ++i)
    indices[i] = i;
  indices[table_description.points.size()] = 0;
  indices[table_description.points.size() + 1] = -1;
  const std::size_t bottom_face_start = table_description.points.size() + 2;

  // bottom face
  for (std::size_t i = 0; i < table_description.points.size(); ++i)
    indices[bottom_face_start + i] = 2 * table_description.points.size() - i - 1;

  indices[bottom_face_start + table_description.points.size()] = table_description.points.size();
  indices[bottom_face_start + table_description.points.size() + 1] = -1;

  std::size_t current_position_in_indices = bottom_face_start + table_description.points.size() + 2;
  // side faces
  for (std::size_t i = 0; i < table_description.points.size(); ++i)
  {
    indices[current_position_in_indices++] = i;
    indices[current_position_in_indices++] = i + table_description.points.size();
    indices[current_position_in_indices++] =
        (i + 1) % table_description.points.size() + table_description.points.size();
    indices[current_position_in_indices++] = (i + 1) % table_description.points.size();
    indices[current_position_in_indices++] = i;
    indices[current_position_in_indices++] = -1;
  }

  auto createTablePolygon = [this, table_description, points, indices](rl::sg::Scene& scene) {
    auto tabletop_model = scene.getModel(table_model_index_);
    for (std::size_t i = 0; i < tabletop_model->getNumBodies(); ++i)
      tabletop_model->remove(tabletop_model->getBody(i));

    auto body = tabletop_model->create();
    auto shape = new SoVRMLShape;
    auto face_set = new SoVRMLIndexedFaceSet;
    auto appearance = new SoVRMLAppearance;
    auto material = new SoVRMLMaterial;
    auto coordinate = new SoVRMLCoordinate;

    material->diffuseColor.setValue(0, 1, 0);
    appearance->material.setValue(material);

    coordinate->point.setValues(0, points.size(), points.data());
    face_set->coordIndex.setValues(0, indices.size(), indices.data());
    face_set->coord.setValue(coordinate);

    shape->appearance.setValue(appearance);
    shape->geometry.setValue(face_set);

    auto sg_shape = body->create(shape);
    sg_shape->setName("table");
    sg_shape->setTransform(rl::math::Transform::Identity());
  };

  createTablePolygon(*bullet_scene_);
  emit applyFunctionToScene(createTablePolygon);
}
