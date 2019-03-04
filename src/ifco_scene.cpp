#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include "utilities.h"
#include "ifco_scene.h"

IfcoScene::IfcoScene(const std::string& scene_graph_file, const std::string& kinematics_file) :
  UsecaseScene(scene_graph_file, kinematics_file)
{
  auto index = findModelIndexByName("ifco");
  if (!index)
  {
    std::stringstream ss;
    ss << "Ifco scene: ifco model not found in " << scene_graph_file;
    throw std::runtime_error(ss.str());
  }

  ifco_model_index_ = *index;
}

void IfcoScene::moveIfco(const rl::math::Transform& ifco_pose)
{
  auto findAndMoveIfco = [this, ifco_pose](rl::sg::Scene& scene) {
    scene.getModel(ifco_model_index_)->getBody(0)->setFrame(ifco_pose);
  };

  findAndMoveIfco(*bullet_scene_);
  emit applyFunctionToScene(findAndMoveIfco);
}
