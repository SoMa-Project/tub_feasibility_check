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

// ========================================================================================== //
Thread::Thread(QObject* parent) :
    QThread(parent), running(false) {
}

// ========================================================================================== //
Thread::~Thread() { }

// ========================================================================================== //
bool Thread::jacobianControl(std::vector<::rl::math::Vector>& steps) {

  model = MainWindow::instance()->model.get();
  static const double delta = 0.017;
  int counter = 0;
  bool reached = false;
  ::rl::math::Vector6 tdot;
  ::rl::math::Vector nextStep = *MainWindow::instance()->start;

  while (!reached) {

    // Update the model
    steps.push_back(nextStep);
    model->setPosition(nextStep);
    model->updateFrames();
    model->updateJacobian();
    model->updateJacobianInverse();

    // Compute the jacobian
    ::rl::math::Transform ee_world = model->forwardPosition();
    ::rl::math::transform::toDelta(ee_world, *MainWindow::instance()->goalFrame, tdot);

    // Compute the velocity
    ::rl::math::Vector qdot(model->getDof());
    qdot.setZero();
    model->inverseVelocity(tdot, qdot);

    // Limit the velocity and decide if reached goal
    if(qdot.norm() < delta)
      return true;
    else {
      qdot.normalize();
      qdot *= delta;
    }

    // Update the configuration
    nextStep = nextStep + qdot;

    // Check for joint limits
    if(!model->isValid(nextStep)) {
      return false;
    }

    // Check for infinite loops
    counter++;
    if(counter > 10.0/delta) {
      std::cout<<"Model left the scene - this should not happen, FIXME!"<<std::endl;
      return false;
    }

    // Check for singularity
    model->setPosition(nextStep);
    model->updateFrames();
    model->updateJacobian();
    model->updateJacobianInverse();
    emit configurationRequested(nextStep);
    usleep(1000);

    if(model->getDof() > 3 && model->getManipulabilityMeasure()  < 1.0e-3f) {
      return false;
    }

    // Check for collision
    model->isColliding();
    auto collisions = model->scene->getLastCollisions();
    if (!collisions.empty())
    {
      // Check if the collision is with a desired object
      for (rl::sg::CollisionMap::iterator it = collisions.begin(); it != collisions.end(); it++)
      {
        // TODO not sure if it's needed to check the reverse pair
        auto reversed_pair = std::make_pair(it->first.second, it->first.first);

        if (!MainWindow::instance()->allowed_collision_pairs.count(it->first) &&
            !MainWindow::instance()->allowed_collision_pairs.count(reversed_pair))
          return false;
      }

      return true;
    }
  }
}

// ========================================================================================== //
void Thread::run() {

  running = true;

  std::vector <::rl::math::Vector> steps;
  bool result = jacobianControl(steps);


  MainWindow::instance()->lastPlanningResult = result;
  MainWindow::instance()->lastTrajectory = steps;

  running = false;

  exit(0);
}

 
// ========================================================================================== //
void Thread::stop() {
  if (running) {
    running = false;
    while (!isFinished()) QThread::usleep(0);
	}
}
