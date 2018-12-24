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

#include <Inventor/Qt/SoQt.h>
#include "MainWindow.h"
#include "Viewer.h"

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) : QMainWindow(parent, f)
{
  // Set up QT stuff
  SoQt::init(this);
  SoDB::init();

  // Set the viewer
  viewer = new Viewer(this);

  // Set window sizes
  int width = 1024;
  int height = 768;
  resize(width, height);
  viewer->setMinimumSize(width, height);

  setAttribute(Qt::WA_QuitOnClose);
}
