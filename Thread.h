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

#ifndef _THREAD_H_
#define _THREAD_H_

#include <QThread>
#include <rl/plan/Viewer.h>
#include <rl/plan/NoisyModel.h>
#include <Inventor/SbColor.h>

class Thread : public QThread
{
	Q_OBJECT
	
public:

	rl::plan::NoisyModel* model;

public:

	Thread(QObject* parent = NULL);
	
	virtual ~Thread();
	
	void reset();
	
	void run();
	
	void stop();
	
  bool jacobianControl(std::vector<::rl::math::Vector>& nextStep);
protected:
	
public:
	bool running;
	
signals:
	void configurationRequested(const rl::math::Vector& q);
  void sphereRequested(const rl::math::Vector& center, const rl::math::Real& radius);
  void boxRequested(const rl::math::Vector& center, const rl::math::Vector& dimensions);
	void colorChangeRequested(const SbColor& col);
};

#endif // _THREAD_H_
