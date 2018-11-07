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

#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
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
	this->viewer = new Viewer(this);
//	this->setCentralWidget(this->viewer);
	
	// Set the physics engine
	this->engine = "bullet";
	
	// Set window sizes
	int width = 1024;
	int height = 768;
	this->resize(width, height);
	this->viewer->setMinimumSize(width, height);

//  std::string path = ros::package::getPath("rlSomaDemo");
  this->rootDir = "/home/ilia/cmp_2/contact-motion-planning";

	// Load the robot model, scene, etc. as well as the CERRT planner
	this->load();
//	this->createPlanner();

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
void MainWindow::plan()
{
  this->model->setPosition(*this->start);
  this->model->updateFrames();
  emit requestConfiguration(*this->start);

  this->thread->stop();
  this->reset();
  this->thread->start();
  usleep(100000);

  while(this->thread->running)
  {
    usleep(100000);
    std::cout<<"waiting for planner"<<std::endl;
  }
}


// ========================================================================================== //
void MainWindow::timerEvent(QTimerEvent *event)
{
  killTimer(timerId);
  //this->thread->start();
}

// ========================================================================================== //
MainWindow::~MainWindow() {
	this->thread->stop();
	MainWindow::singleton = NULL;
}

// ========================================================================================== //
void MainWindow::processArgs () {
	QRegExp exp1("--rootDir");
	QRegExp exp2("--joints");
	QRegExp exp3("--help");
	QRegExp exp4("--goalFrame");
	QRegExp exp5("--collObj");
	QRegExp exp6("--problemID");
	for (int i = 1; i < QApplication::arguments().size(); ++i) {

    if(-1 != exp1.indexIn(QApplication::arguments()[i])) {
      this->rootDir = QApplication::arguments()[i+1].toStdString();
    }else{
      this->rootDir = "/home/arne/projects/soma_ws/src/contact-motion-planning";
    }

		if(-1 != exp2.indexIn(QApplication::arguments()[i])) {
      std::string joints = QApplication::arguments()[i+1].toStdString();
      std::istringstream ss (joints); 
      std::string token;
      // TODO: Process this after the model is loaded so you don't have to assume 7 joints
      this->start = boost::make_shared< rl::math::Vector >(7);
      int counter = 0;
      while(std::getline(ss, token, ',')) {
        (*this->start)(counter++) = std::atof(token.c_str());
      }
      std::cout << "startJoint: '" << joints << "'\n\tparsed as " << (*this->start).transpose() << std::endl;
    }

		if(-1 != exp3.indexIn(QApplication::arguments()[i])) {
      std::cout << "Example usage: ./demos/rlSomaDemo/rlSomaDemod --rootDir /home/cerdogan/Documents/ros/src/contact-motion-planning --hide --joints 0.9,0.9,0.3,0.5,0.6,0.3,0.2 --goalFrame 0.690,-0.010,0.684,0.681,0.000,0.733,-0.000 (quaternion: x,y,z,w)" << std::endl;
      exit(1);
		}
    if(-1 != exp4.indexIn(QApplication::arguments()[i])) {
      std::string frame = QApplication::arguments()[i+1].toStdString();
      std::istringstream ss (frame); 
      std::string token;
      rl::math::Vector posQuat (7);
      int counter = 0;
      while(std::getline(ss, token, ',')) {
        posQuat(counter++) = std::atof(token.c_str());
      }
      std::cout << "goalFrame: '" << frame << "'\n\tparsed as " << posQuat.transpose() << std::endl;

      // Create a frame from the position/quaternion data
      this->goalFrame = boost::make_shared< rl::math::Transform >();
      Eigen::Quaternion <double> q (posQuat(6), posQuat(3), posQuat(4), posQuat(5)); 
      this->goalFrame->linear() = q.toRotationMatrix();
      this->goalFrame->translation() = Eigen::Vector3d(posQuat(0), posQuat(1), posQuat(2));
		}
    if(-1 != exp5.indexIn(QApplication::arguments()[i])) {
      this->desiredCollObj = "_" + QApplication::arguments()[i+1].toStdString();
      pc(this->desiredCollObj);
		}
    if(-1 != exp6.indexIn(QApplication::arguments()[i])) {
      this->problemID = QApplication::arguments()[i+1].toStdString();
      pc(this->problemID);
		}
  }

}

// ========================================================================================== //
void MainWindow::clear() {
	this->kin.reset();
	this->model.reset();
	this->scene.reset();
	this->sceneModel = NULL;
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
	
	this->clear();

	// Load kinematics of the robot
	this->kin.reset(rl::kin::Kinematics::create(
		this->rootDir + "/rl-examples-0.6.2/rlkin/barrett-wam-ocado2.xml"));
  kin2.reset(rl::kin::Kinematics::create(rootDir + "/rl-examples-0.6.2/rlkin/barrett-wam-ocado2.xml"));
	// this->kin->world().translation().z() = -0.195;
	
	// Load the scene
	this->scene = boost::make_shared< rl::sg::bullet::Scene >();
	this->scene->load(this->rootDir + "/rl-examples-0.6.2/rlsg/barrett-wam-wrist_wall_real.convex.xml");

	this->sceneModel = static_cast< rl::sg::bullet::Model* >(this->scene->getModel(0));
	
	// Create the model for the problem
	this->model = boost::make_shared< rl::plan::NoisyModel >();
	this->model->kin = this->kin.get();
	this->model->model = this->sceneModel;
	this->model->scene = this->scene.get();
	
	// Load the visual scene
	this->visScene = boost::make_shared< rl::sg::so::Scene >();
	this->visScene->load(this->rootDir + "/rl-examples-0.6.2/rlsg/barrett-wam-wrist_wall_real.convex.xml");
	this->visSceneModel = static_cast< rl::sg::so::Model* >(this->visScene->getModel(0));
	
	// Create the model for the visualization
	this->visModel = boost::make_shared< rl::plan::NoisyModel >();
  this->visModel->kin = kin2.get();
	this->visModel->model = this->visSceneModel;
	this->visModel->scene = this->visScene.get();

	// Setup the viewer
	this->viewer->sceneGroup->addChild(this->visScene->root);
	this->viewer->model = this->visModel.get();
	this->toggleView(false);
	this->viewer->viewer->setBackgroundColor(SbColor(1,1,1));
	this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
	this->viewer->viewer->getCamera()->setToDefaults();
	this->viewer->viewer->viewAll();
}

// ========================================================================================== //
void MainWindow::reset() {

//	this->thread->blockSignals(true);
//	QCoreApplication::processEvents();
//	this->thread->stop();
//	this->thread->blockSignals(false);
	
	this->model->reset();
	this->visModel->reset();
  resetViewer();
}

// ========================================================================================== //
void MainWindow::toggleView(const bool& doOn) {
}

// ========================================================================================== //
void MainWindow::handleGUI(int i) {
}
// ========================================================================================== //
