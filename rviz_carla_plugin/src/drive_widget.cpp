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

#include <math.h>
#include <stdio.h>

#include <QMouseEvent>
#include <QPainter>

#include "drive_widget.h"

namespace rviz_carla_plugin {

// BEGIN_TUTORIAL
// The DriveWidget constructor does the normal Qt thing of
// passing the parent widget to the superclass constructor, then
// initializing the member variables.
DriveWidget::DriveWidget(QWidget *parent)
  : QWidget(parent)
  , linear_velocity_(0)
  , angular_velocity_(0)
  , linear_scale_(10)
  , angular_scale_(.2)
{
}

// This paintEvent() is complex because of the drawing of the two
// arc-arrows representing wheel motion.  It is not particularly
// relevant to learning how to make an RViz plugin, so I will kind of
// skim it.
void DriveWidget::paintEvent(QPaintEvent *event)
{
  // The background color and crosshair lines are drawn differently
  // depending on whether this widget is enabled or not.  This gives a
  // nice visual indication of whether the control is "live".
  QColor background;
  QColor crosshair;
  if (isEnabled())
  {
    background = Qt::white;
    crosshair = Qt::black;
  }
  else
  {
    background = Qt::lightGray;
    crosshair = Qt::darkGray;
  }

  // The main visual is a square, centered in the widget's area.  Here
  // we compute the size of the square and the horizontal and vertical
  // offsets of it.
  int w = width();
  int h = height();
  int size = ((w > h) ? h : w) - 1;
  int hpad = (w - size) / 2;
  int vpad = (h - size) / 2;

  QPainter painter(this);
  painter.setBrush(background);
  painter.setPen(crosshair);

  // Draw the background square.
  painter.drawRect(QRect(hpad, vpad, size, size));

  // Draw a cross-hair inside the square.
  painter.drawLine(hpad, height() / 2, hpad + size, height() / 2);
  painter.drawLine(width() / 2, vpad, width() / 2, vpad + size);

  // If the widget is enabled and the velocities are not zero, draw
  // some sweet green arrows showing possible paths that the wheels of
  // a diff-drive robot would take if it stayed at these velocities.
  if (isEnabled() && (angular_velocity_ != 0 || linear_velocity_ != 0))
  {
    QPen arrow;
    arrow.setWidth(size / 20);
    arrow.setColor(Qt::green);
    arrow.setCapStyle(Qt::RoundCap);
    arrow.setJoinStyle(Qt::RoundJoin);
    painter.setPen(arrow);

    // This code steps along a central arc defined by the linear and
    // angular velocites.  At each step, it computes where the left
    // and right wheels would be and collects the resulting points
    // in the left_track and right_track arrays.
    const int step_count = 100;
    QPointF left_track[step_count];
    QPointF right_track[step_count];

    float half_track_width = size / 4.0;

    float cx = w / 2;
    float cy = h / 2;
    left_track[0].setX(cx - half_track_width);
    left_track[0].setY(cy);
    right_track[0].setX(cx + half_track_width);
    right_track[0].setY(cy);
    float angle = M_PI / 2;
    float delta_angle = angular_velocity_ / step_count;
    float step_dist = linear_velocity_ * size / 2 / linear_scale_ / step_count;
    for (int step = 1; step < step_count; step++)
    {
      angle += delta_angle / 2;
      float next_cx = cx + step_dist * cosf(angle);
      float next_cy = cy - step_dist * sinf(angle);
      angle += delta_angle / 2;

      left_track[step].setX(next_cx + half_track_width * cosf(angle + M_PI / 2));
      left_track[step].setY(next_cy - half_track_width * sinf(angle + M_PI / 2));
      right_track[step].setX(next_cx + half_track_width * cosf(angle - M_PI / 2));
      right_track[step].setY(next_cy - half_track_width * sinf(angle - M_PI / 2));

      cx = next_cx;
      cy = next_cy;
    }
    // Now the track arrays are filled, so stroke each with a fat green line.
    painter.drawPolyline(left_track, step_count);
    painter.drawPolyline(right_track, step_count);

    // Here we decide which direction each arrowhead will point
    // (forward or backward).  This works by comparing the arc length
    // travelled by the center in one step (step_dist) with the arc
    // length travelled by the wheel (half_track_width * delta_angle).
    int left_arrow_dir = (-step_dist + half_track_width * delta_angle > 0);
    int right_arrow_dir = (-step_dist - half_track_width * delta_angle > 0);

    // Use MiterJoin for the arrowheads so we get a nice sharp point.
    arrow.setJoinStyle(Qt::MiterJoin);
    painter.setPen(arrow);

    // Compute and draw polylines for each arrowhead.  This code could
    // probably be more elegant.
    float head_len = size / 8.0;
    QPointF arrow_head[3];
    float x, y;
    if (fabsf(-step_dist + half_track_width * delta_angle) > .01)
    {
      x = left_track[step_count - 1].x();
      y = left_track[step_count - 1].y();
      arrow_head[0].setX(x + head_len * cosf(angle + 3 * M_PI / 4 + left_arrow_dir * M_PI));
      arrow_head[0].setY(y - head_len * sinf(angle + 3 * M_PI / 4 + left_arrow_dir * M_PI));
      arrow_head[1].setX(x);
      arrow_head[1].setY(y);
      arrow_head[2].setX(x + head_len * cosf(angle - 3 * M_PI / 4 + left_arrow_dir * M_PI));
      arrow_head[2].setY(y - head_len * sinf(angle - 3 * M_PI / 4 + left_arrow_dir * M_PI));
      painter.drawPolyline(arrow_head, 3);
    }
    if (fabsf(-step_dist - half_track_width * delta_angle) > .01)
    {
      x = right_track[step_count - 1].x();
      y = right_track[step_count - 1].y();
      arrow_head[0].setX(x + head_len * cosf(angle + 3 * M_PI / 4 + right_arrow_dir * M_PI));
      arrow_head[0].setY(y - head_len * sinf(angle + 3 * M_PI / 4 + right_arrow_dir * M_PI));
      arrow_head[1].setX(x);
      arrow_head[1].setY(y);
      arrow_head[2].setX(x + head_len * cosf(angle - 3 * M_PI / 4 + right_arrow_dir * M_PI));
      arrow_head[2].setY(y - head_len * sinf(angle - 3 * M_PI / 4 + right_arrow_dir * M_PI));
      painter.drawPolyline(arrow_head, 3);
    }
  }
}

// Every mouse move event received here sends a velocity because Qt
// only sends us mouse move events if there was previously a
// mouse-press event while in the widget.
void DriveWidget::mouseMoveEvent(QMouseEvent *event)
{
  sendVelocitiesFromMouse(event->x(), event->y(), width(), height());
}

// Mouse-press events should send the velocities too, of course.
void DriveWidget::mousePressEvent(QMouseEvent *event)
{
  sendVelocitiesFromMouse(event->x(), event->y(), width(), height());
}

// When the mouse leaves the widget but the button is still held down,
// we don't get the leaveEvent() because the mouse is "grabbed" (by
// default from Qt).  However, when the mouse drags out of the widget
// and then other buttons are pressed (or possibly other
// window-manager things happen), we will get a leaveEvent() but not a
// mouseReleaseEvent().  Without catching this event you can have a
// robot stuck "on" without the user controlling it.
void DriveWidget::leaveEvent(QEvent *event)
{
  stop();
}

// The ordinary way to stop: let go of the mouse button.
void DriveWidget::mouseReleaseEvent(QMouseEvent *event)
{
  stop();
}

// Compute and emit linear and angular velocities based on Y and X
// mouse positions relative to the central square.
void DriveWidget::sendVelocitiesFromMouse(int x, int y, int width, int height)
{
  int size = ((width > height) ? height : width);
  int hpad = (width - size) / 2;
  int vpad = (height - size) / 2;

  linear_velocity_ = (1.0 - float(y - vpad) / float(size / 2)) * linear_scale_;
  angular_velocity_ = (1.0 - float(x - hpad) / float(size / 2)) * angular_scale_;
  Q_EMIT outputVelocity(linear_velocity_, angular_velocity_);

  // update() is a QWidget function which schedules this widget to be
  // repainted the next time through the main event loop.  We need
  // this because the velocities have just changed, so the arrows need
  // to be redrawn to match.
  update();
}

// How to stop: emit velocities of 0!
void DriveWidget::stop()
{
  linear_velocity_ = 0;
  angular_velocity_ = 0;
  Q_EMIT outputVelocity(linear_velocity_, angular_velocity_);
  update();
}
// END_TUTORIAL

} // end namespace rviz_carla_plugin
