#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
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

void TabletopScene::createTable(const TableDescription& table_description)
{
  std::vector<SbVec3f> points(table_description.points.size());
  std::vector<int32_t> indices(points.size() + 2);

  for (std::size_t i = 0; i < points.size(); ++i)
    points[i].setValue(table_description.points[i].x(), table_description.points[i].y(),
                       table_description.points[i].z());

  for (std::size_t i = 0; i < points.size(); ++i)
    indices[i] = i;
  indices[points.size()] = 0;
  indices[points.size() + 1] = -1;

  auto createTablePolygon = [this, table_description, points, indices](rl::sg::Scene& scene) {
    auto body = scene.getModel(table_model_index_)->create();
    auto shape = new SoVRMLShape;
    auto face_set = new SoVRMLIndexedFaceSet;
    auto appearance = new SoVRMLAppearance;
    auto material = new SoVRMLMaterial;
    auto coordinate = new SoVRMLCoordinate;

    material->diffuseColor.setValue(1, 0, 0);
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
