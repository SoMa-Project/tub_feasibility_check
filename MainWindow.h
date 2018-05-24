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

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QAction>
#include <QDockWidget>
#include <QGraphicsView>
#include <QMainWindow>
#include <QMutex>
#include <QTableView>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/mdl/Dynamic.h>
#include <rl/plan/NoisyModel.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Model.h>
#include <rl/sg/Scene.h>
#include <rl/sg/bullet/Model.h>
#include <rl/sg/bullet/Scene.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/so/Scene.h>

class Thread;
class Viewer;

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	virtual ~MainWindow();
	
	static MainWindow* instance();
	
	boost::shared_ptr< rl::kin::Kinematics > kin;
	
	boost::shared_ptr< rl::plan::NoisyModel > model;
	boost::shared_ptr< rl::plan::NoisyModel > visModel;

	QMutex mutex;
	
	boost::shared_ptr< rl::sg::bullet::Scene > scene;
	rl::sg::bullet::Model* sceneModel;

	boost::shared_ptr< rl::sg::so::Scene > visScene; // Due to VRML object not in bullet
	rl::sg::so::Model* visSceneModel;
	
	boost::shared_ptr< rl::plan::Planner > planner;
	boost::shared_ptr< rl::math::Vector > start;
	boost::shared_ptr< rl::math::Vector > goal;
	boost::shared_ptr< rl::math::Transform > goalFrame;
  std::string desiredCollObj;
  std::string problemID;    // An identifier for the problem for documentation

  bool lastPlanningResult;
  std::vector <::rl::math::Vector> lastTrajectory;

	Thread* thread;
	
	Viewer* viewer;

	bool wait;	// whether the planner should start immediately without SPACE and quite thereafter
  std::string rootDir; // where the rl-examples... folder lives

public slots:
	
	void reset();
	
	void toggleView(const bool& doOn);
	
	/// Handles actions based on the action types (defined below)
	void handleGUI(int i);

  void plan();

protected:
	MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);
  void timerEvent (QTimerEvent* event);
	
private:
  int timerId;

	void clear();
	
	void connect(const QObject* sender, const QObject* receiver);
	
	void disconnect(const QObject* sender, const QObject* receiver);
	
	void init();
	
	void load();

  /// Performs jacobian control to go from the initial joint configuration to the given task frame
  /// TODO: When does it return true?
  //bool jacobianControl();
	
	/// Processes input arguments to the program 
	void processArgs ();

	/// Reads the geometry and the connectivity of the polygons/planes from an .xml file
	void readPlanes();

	/// Initializes the CERRT planner with hardcoded constants
	void createPlanner();	

	QString engine;
	
	static MainWindow* singleton;
	
	std::vector <QAction*> qactions; //

	enum ActionType {
		StartThread = 0,
		ViewPath,
		ViewPathNext,
		ViewPathPrev
	};
	struct AD {
		std::string s;
		ActionType t;
		AD(const std::string& str, ActionType type) : s(str), t(type) {}
	};

};

#endif // _MAINWINDOW_H_
