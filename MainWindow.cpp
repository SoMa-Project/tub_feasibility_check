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

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) : QMainWindow(parent, f)
{
  // Set up QT stuff
  MainWindow::singleton = this;
  SoQt::init(this);
  SoDB::init();

  // Set the viewer
  viewer = new Viewer(this);

  // Set window sizes
  int width = 1024;
  int height = 768;
  resize(width, height);
  viewer->setMinimumSize(width, height);
}

MainWindow::~MainWindow()
{
  MainWindow::singleton = NULL;
}

MainWindow* MainWindow::instance()
{
  if (NULL == MainWindow::singleton)
    new MainWindow();
  return MainWindow::singleton;
}
