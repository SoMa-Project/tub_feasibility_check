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

#include <fstream>
#include <QApplication>
#include <QDateTime>
#include <QMutexLocker>
#include <rl/math/Quaternion.h>
#include <rl/math/Unit.h>
#include <rl/util/Timer.h>

#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

#define pc(x) std::cout << #x << ": "  << (x) << std::endl;
#define pv(x) std::cout << #x << ": "  << (x).transpose() << std::endl;
#define ps(x) std::cout << x << std::endl;

// ========================================================================================== //
Thread::Thread(QObject* parent) :
		QThread(parent), quit(false), swept(false), running(false), operation(TEST_PLAN), 
		pathIndex(0) {
}

// ========================================================================================== //
Thread::~Thread() { }

// ========================================================================================== //
bool Thread::jacobianControl(std::vector<::rl::math::Vector>& steps) {

  this->model = MainWindow::instance()->model.get();
  static const double delta = 0.017;
  int counter = 0;
  bool reached = false;
  ::rl::math::Vector6 tdot;
  ::rl::math::Vector nextStep = *MainWindow::instance()->start;
  ::rl::sg::CollisionMap allColls;

  while (!reached) {

    // Update the model
    steps.push_back(nextStep);
    this->model->setPosition(nextStep);
    this->model->updateFrames();
    this->model->updateJacobian();
    this->model->updateJacobianInverse();

    // Compute the jacobian
    ::rl::math::Transform ee_world = this->model->forwardPosition();
    ::rl::math::transform::toDelta(ee_world, *MainWindow::instance()->goalFrame, tdot);

    // Compute the velocity
    ::rl::math::Vector qdot(this->model->getDof());
    qdot.setZero();
    this->model->inverseVelocity(tdot, qdot);

    // Limit the velocity and decide if reached goal
    if(qdot.norm() < delta) {
      reached = true;
      ps("Reached the goal :)");
      return true;
    }
    else {
      qdot.normalize();
      qdot *= delta;
    }

    // Update the configuration
    nextStep = nextStep + qdot;

    // Check for joint limits
    if(!this->model->isValid(nextStep)) {
      ps("Hit joint limit :(");
      return false;
    }

    // Check for infinite loops
    counter++;
    if(counter > 10.0/delta) {
      std::cout<<"Model left the scene - this should not happen, FIXME!"<<std::endl;
      ps("Stuck in loop :(");
      return false;
    }

    // Check for singularity
    this->model->setPosition(nextStep);
    this->model->updateFrames();
    this->model->updateJacobian();
    this->model->updateJacobianInverse();
    emit configurationRequested(nextStep);
    usleep(1000);

    if(this->model->getDof() > 3 && this->model->getManipulabilityMeasure()  < 1.0e-3f) {
      ps("Hit singularity :(");
      return false;
    }

    // Check for collision
    this->model->isColliding();
    allColls = this->model->scene->getLastCollisions();
    if(!allColls.empty()) {
      // Check if the collision is with a desired object
      for(rl::sg::CollisionMap::iterator it = allColls.begin(); it != allColls.end(); it++) { 

        // ps(it->first.first); ps(it->first.second);
        if((MainWindow::instance()->desiredCollObj.compare(it->first.first) == 0) || 
            (MainWindow::instance()->desiredCollObj.compare(it->first.second) == 0)) {
          ps("Collision with desired object :)");
          return true;
        }
      }
      ps("Collision with bad object :(");
      return false;
    }

  }
}

// ========================================================================================== //
void Thread::run() {

  this->running = true;

  std::vector <::rl::math::Vector> steps;
  bool result = jacobianControl(steps);


  MainWindow::instance()->lastPlanningResult = result;
  MainWindow::instance()->lastTrajectory = steps;

  this->running = false;

  pc(result);

  // Write the result to a file
  ::rl::math::Vector lastStep = steps.back();
  FILE* file = fopen("collision-result.txt", "w");
  fprintf(file, "%d\n", result);
  for(int i = 0; i < 7; i++) fprintf(file, "%f ", lastStep(i));
  fprintf(file, "\n");
  fclose(file);

  // Write the trajectory to a different file
  std::ofstream ofs (MainWindow::instance()->problemID + ".traj", std::ofstream::out);
  for(int i = 0; i < steps.size(); i++) 
    ofs << steps[i].transpose() << "\n";
  ofs.close();

  exit(0);
}

 
// ========================================================================================== //
void Thread::stop() {
	if (this->running) {
		this->running = false;
		while (!this->isFinished()) QThread::usleep(0);
	}
}
