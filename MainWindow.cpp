//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

/**
 * @author Can Erdogan
 * @date 2017-05-29
 * @brief Shortened version of rlPlanDemo dedicated to Jacobian control AND
 * reads the plane information from an .xml file AND
 * reimplements moveConfigOntoSurface function to explicitly assume plane information
 * and speed up collision checking.
 */

#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QGLWidget>
#include <QGraphicsView>
#include <QHeaderView>
#include <QLayout>
#include <QMenuBar>
#include <QMutexLocker>
#include <QPainter>
#include <QPrinter>
#include <QRegExp>
#include <QSignalMapper>
#include <boost/make_shared.hpp>

#include "ros/ros.h"
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>

#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/VRMLnodes/SoVRMLBox.h>
#include <Inventor/Qt/SoQt.h>

#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/sg/Body.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/plan/UniformSampler.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Path.h>

#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

#include <iostream>
#include <sstream>

#define pc(x) std::cout << #x << ": "  << (x) << std::endl;
#define pv(x) std::cout << #x << ": "  << (x).transpose() << std::endl;
#define ps(x) std::cout << x << std::endl;

// ========================================================================================== //
MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) :
		QMainWindow(parent, f),
		kin(),
		model(),
		scene(),
		sceneModel(NULL),
		thread(new Thread(this)),
		viewer(NULL),
		engine(), wait(true) {

	// Set up QT stuff
	MainWindow::singleton = this;
	SoQt::init(this);
	SoDB::init();
	
	// Set the viewer
  viewer = new Viewer(this);
//	setCentralWidget(viewer);
	
	// Set the physics engine
  engine = "bullet";
	
	// Set window sizes
	int width = 1024;
	int height = 768;
  resize(width, height);
  viewer->setMinimumSize(width, height);

//  std::string path = ros::package::getPath("rlSomaDemo");
  rootDir = "/home/ilia/cmp_2/contact-motion-planning";

	// Load the robot model, scene, etc. as well as the CERRT planner
  load();
//	createPlanner();

  timerId = startTimer(100);
  connect(thread, viewer);
  QObject::connect(this, SIGNAL(requestConfiguration(const rl::math::Vector&)), viewer,
                   SLOT(drawConfiguration(const rl::math::Vector&)));
  QObject::connect(this, SIGNAL(requestBox(const rl::math::Vector&, const rl::math::Transform&)),
                   viewer, SLOT(drawBox(const rl::math::Vector&, const rl::math::Transform&)));
  QObject::connect(this, SIGNAL(requestResetViewer()), viewer, SLOT(reset()));
  QObject::connect(this, SIGNAL(requestResetViewerBoxes()), viewer, SLOT(resetBoxes()));
}

// ========================================================================================== //
void MainWindow::plan(const rl::math::Transform& ifco_transform,
                      const std::vector<kinematics_check::BoundingBoxWithPose>& boundingBoxes)
{
  rl::sg::Model* ifco;
  rl::sg::Model* ifco2;
  for (int i = 0; i < scene->getNumModels(); ++i)
  {
    auto model = scene->getModel(i);
    if (model->getName() == "ifco")
    {
      ifco = model;
      ifco2 = visScene->getModel(i);
      break;
    }
  }

  ifco->getBody(0)->setFrame(ifco_transform);
  ifco2->getBody(0)->setFrame(ifco_transform);

  for (int i = 1; i < ifco->getNumBodies(); ++i)
  {
    ifco->remove(ifco->getBody(i));
    ifco2->remove(ifco2->getBody(i));
  }

  for (int i = 0; i < boundingBoxes.size(); ++i)
  {
    auto& boundingBox = boundingBoxes[i];

    auto boundingBoxBodies = std::make_pair(ifco->create(), ifco2->create());
    auto shape = new SoVRMLShape;
    auto appearance = new SoVRMLAppearance;
    auto material = new SoVRMLMaterial;
    auto box = new SoVRMLBox;
    material->diffuseColor.setValue(0, 1, 0);
    material->transparency.setValue(0.5);
    appearance->material.setValue(material);
    shape->appearance.setValue(appearance);
    std::stringstream ss;
    ss << "box" << i + 1;
    boundingBoxBodies.first->setName(ss.str());
    boundingBoxBodies.second->setName(ss.str());

    box->size.setValue(boundingBox.box.dimensions[0], boundingBox.box.dimensions[1], boundingBox.box.dimensions[2]);
    shape->geometry.setValue(box);

    auto sgShapes = std::make_pair(boundingBoxBodies.first->create(shape), boundingBoxBodies.second->create(shape));

    Eigen::Affine3d box_transform;
    tf::poseMsgToEigen(boundingBox.pose, box_transform);
    sgShapes.first->setTransform(box_transform);
    sgShapes.second->setTransform(box_transform);

    boundingBoxBodies.first->add(sgShapes.first);
    boundingBoxBodies.second->add(sgShapes.second);
  }

  model->setPosition(*start);
  model->updateFrames();
  emit requestConfiguration(*start);

  thread->stop();
  reset();
  thread->start();
  usleep(100000);

  while(thread->running)
  {
    usleep(100000);
    std::cout<<"waiting for planner"<<std::endl;
  }
}


// ========================================================================================== //
void MainWindow::timerEvent(QTimerEvent *event)
{
  killTimer(timerId);
  //thread->start();
}

// ========================================================================================== //
MainWindow::~MainWindow() {
  thread->stop();
	MainWindow::singleton = NULL;
}

// ========================================================================================== //
void MainWindow::clear() {
  kin.reset();
  model.reset();
  scene.reset();
  sceneModel = NULL;
}

// ========================================================================================== //
void MainWindow::connect(const QObject* sender, const QObject* receiver) {
  QObject::connect(
        sender,
        SIGNAL(configurationRequested(const rl::math::Vector&)),
        receiver,
        SLOT(drawConfiguration(const rl::math::Vector&))
        );

  QObject::connect(
        sender,
        SIGNAL(sphereRequested(const rl::math::Vector&, const rl::math::Real&)),
        receiver,
        SLOT(drawSphere(const rl::math::Vector&, const rl::math::Real&))
        );

  QObject::connect(
        sender,
        SIGNAL(colorChangeRequested(const SbColor&)),
        receiver,
        SLOT(changeColor(const SbColor&))
        );
}

// ========================================================================================== //
void MainWindow::disconnect(const QObject* sender, const QObject* receiver) {
	QObject::disconnect(sender, NULL, receiver, NULL);
}

// ========================================================================================== //
MainWindow* MainWindow::instance() {
	if (NULL == MainWindow::singleton) new MainWindow();
  return MainWindow::singleton;
}

void MainWindow::drawBox(const rl::math::Vector &size, const rl::math::Transform &transform)
{
  emit requestBox(size, transform);
}

void MainWindow::resetViewer()
{
  emit requestResetViewer();
}

void MainWindow::resetViewerBoxes()
{
  emit requestResetViewerBoxes();
}

// ========================================================================================== //
void MainWindow::load() {
	
  clear();

	// Load kinematics of the robot
  kin.reset(rl::kin::Kinematics::create(
    rootDir + "/soma/rlkin/barrett-wam-ocado2.xml"));
  kin2.reset(rl::kin::Kinematics::create(rootDir + "/soma/rlkin/barrett-wam-ocado2.xml"));
  // kin->world().translation().z() = -0.195;
	
	// Load the scene
  scene = boost::make_shared< rl::sg::bullet::Scene >();
  scene->load(rootDir + "/soma/rlsg/wam-rbohand-ifco.convex.xml");

  sceneModel = static_cast< rl::sg::bullet::Model* >(scene->getModel(0));
	
	// Create the model for the problem
  model = boost::make_shared< rl::plan::NoisyModel >();
  model->kin = kin.get();
  model->model = sceneModel;
  model->scene = scene.get();
	
	// Load the visual scene
  visScene = boost::make_shared< rl::sg::so::Scene >();
  visScene->load(rootDir + "/soma/rlsg/wam-rbohand-ifco.convex.xml");
  visSceneModel = static_cast< rl::sg::so::Model* >(visScene->getModel(0));
	
	// Create the model for the visualization
  visModel = boost::make_shared< rl::plan::NoisyModel >();
  visModel->kin = kin2.get();
  visModel->model = visSceneModel;
  visModel->scene = visScene.get();

	// Setup the viewer
  viewer->sceneGroup->addChild(visScene->root);
  viewer->model = visModel.get();
  viewer->viewer->setBackgroundColor(SbColor(1,1,1));
  viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
  viewer->viewer->getCamera()->setToDefaults();
  viewer->viewer->viewAll();
}

// ========================================================================================== //
void MainWindow::reset() {

//	thread->blockSignals(true);
//	QCoreApplication::processEvents();
//	thread->stop();
//	thread->blockSignals(false);
	
  model->reset();
  visModel->reset();
  resetViewer();
}
