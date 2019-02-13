#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include "utilities.h"
#include "ifco_scene.h"

IfcoScene::IfcoScene(const std::string& scene_graph_file, const std::string& kinematics_file) :
  UsecaseScene(scene_graph_file, kinematics_file)
{
  for (std::size_t i = 0; i < bullet_scene_->getNumModels(); ++i)
  {
    auto model = bullet_scene_->getModel(i);
    if (model->getName() == "ifco")
    {
      ifco_model_index_ = i;
      break;
    }
  }
}

void IfcoScene::moveIfco(const rl::math::Transform& ifco_pose)
{
  auto findAndMoveIfco = [this, ifco_pose](rl::sg::Scene& scene) {
    scene.getModel(ifco_model_index_)->getBody(0)->setFrame(ifco_pose);
  };

  findAndMoveIfco(*bullet_scene_);
  emit applyFunctionToScene(findAndMoveIfco);
}
