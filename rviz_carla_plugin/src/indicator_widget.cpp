/*
 * Copyright (c) 2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include "indicator_widget.h"
#include <QPainter>

namespace rviz_carla_plugin {

IndicatorWidget::IndicatorWidget(QWidget *parent)
  : QWidget(parent)
  , mCurrentState(IndicatorWidget::State::Stopped)
{
  setFixedSize(18, 18);
}

void IndicatorWidget::paintEvent(QPaintEvent *event)
{
  QPainter painter(this);
  painter.setPen(Qt::darkGray);
  if (mCurrentState == IndicatorWidget::State::Stopped)
  {
    painter.setBrush(QBrush(Qt::lightGray, Qt::SolidPattern));
  }
  else if (mCurrentState == IndicatorWidget::State::Running)
  {
    painter.setBrush(QBrush(Qt::green, Qt::SolidPattern));
  }
  else if (mCurrentState == IndicatorWidget::State::Error)
  {
    painter.setBrush(QBrush(Qt::red, Qt::SolidPattern));
  }
  else if ((mCurrentState == IndicatorWidget::State::Starting)
           || (mCurrentState == IndicatorWidget::State::ShuttingDown))
  {
    painter.setBrush(QBrush(Qt::yellow, Qt::SolidPattern));
  }
  else
  {
    painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));
  }
  painter.drawEllipse(1, 1, 16, 16);
}

void IndicatorWidget::setState(IndicatorWidget::State state)
{
  mCurrentState = state;
  repaint();
}

} // end namespace rviz_carla_plugin
