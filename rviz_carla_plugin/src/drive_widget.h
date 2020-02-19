/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef DRIVE_WIDGET_H
#define DRIVE_WIDGET_H

#include <QWidget>

namespace rviz_carla_plugin {

// BEGIN_TUTORIAL
// DriveWidget implements a control which translates mouse Y values
// into linear velocities and mouse X values into angular velocities.
//
// For maximum reusability, this class is only responsible for user
// interaction and display inside its widget.  It does not make any
// ROS or RViz calls.  It communicates its data to the outside just
// via Qt signals.
class DriveWidget : public QWidget
{
  Q_OBJECT
public:
  // This class is not instantiated by pluginlib::ClassLoader, so the
  // constructor has no restrictions.
  DriveWidget(QWidget *parent = 0);

  // We override QWidget::paintEvent() to do custom painting.
  virtual void paintEvent(QPaintEvent *event);

  // We override the mouse events and leaveEvent() to keep track of
  // what the mouse is doing.
  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseReleaseEvent(QMouseEvent *event);
  virtual void leaveEvent(QEvent *event);

  // Override sizeHint() to give the layout managers some idea of a
  // good size for this.
  virtual QSize sizeHint() const
  {
    return QSize(150, 150);
  }

  // We emit outputVelocity() whenever it changes.
Q_SIGNALS:
  void outputVelocity(float linear, float angular);

  // mouseMoveEvent() and mousePressEvent() need the same math to
  // figure the velocities, so I put that in here.
protected:
  void sendVelocitiesFromMouse(int x, int y, int width, int height);

  // A function to emit zero velocity.
  void stop();

  // Finally the member variables:
  float linear_velocity_;  // In m/s
  float angular_velocity_; // In radians/s
  float linear_scale_;     // In m/s
  float angular_scale_;    // In radians/s
};
// END_TUTORIAL

} // end namespace rviz_carla_plugin

#endif // DRIVE_WIDGET_H
