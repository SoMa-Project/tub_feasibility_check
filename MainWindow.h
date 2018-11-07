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
#include <unordered_set>

#include "kinematics_check/BoundingBoxWithPose.h"

class Thread;
class Viewer;

struct PairHash
{
  template <class T1, class T2>
  std::size_t operator() (const std::pair<T1, T2>& p) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, p.first);
    boost::hash_combine(seed, p.second);

    return seed;
  }
};

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	virtual ~MainWindow();
	
	static MainWindow* instance();
  void drawBox(const rl::math::Vector& size, const rl::math::Transform& transform);
  void resetViewer();
  void resetViewerBoxes();

	boost::shared_ptr< rl::kin::Kinematics > kin;
  boost::shared_ptr< rl::kin::Kinematics > kin2;

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
  std::unordered_set<std::pair<std::string, std::string>, PairHash> allowed_collision_pairs;
  std::string problemID;    // An identifier for the problem for documentation

  bool lastPlanningResult;
  std::vector <::rl::math::Vector> lastTrajectory;

	Thread* thread;
	
	Viewer* viewer;

	bool wait;	// whether the planner should start immediately without SPACE and quite thereafter
  std::string rootDir; // where the rl-examples... folder lives

public slots:
	
	void reset();
	
  void plan(const rl::math::Transform& ifco_transform,
            const std::vector<kinematics_check::BoundingBoxWithPose>& boundingBoxesWithPoses);

protected:
	MainWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);
  void timerEvent (QTimerEvent* event);
	
private:
  int timerId;

	void clear();
	
	void connect(const QObject* sender, const QObject* receiver);
	
	void disconnect(const QObject* sender, const QObject* receiver);
	
	void load();

	QString engine;
	
	static MainWindow* singleton;

signals:
  void requestConfiguration(const rl::math::Vector& q);
  void requestBox(const rl::math::Vector& size, const rl::math::Transform& transform);
  void requestResetViewer();
  void requestResetViewerBoxes();
};

#endif // _MAINWINDOW_H_
