/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef NTK_GUI_IMAGEWIDGET_H
#define NTK_GUI_IMAGEWIDGET_H

#include <ntk/core.h>

#include <QPoint>
#include <QImage>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;

namespace ntk
{

class ImageWidget : public QWidget
{
  Q_OBJECT

public:
  ImageWidget(QWidget* parent) : QWidget(parent), m_last_mouse_pos(-1,-1)
  {}

public:
  double scaleX() const;
  double scaleY() const;

  void getLastMousePos(int& x, int& y)
  { x = m_last_mouse_pos.x(); y = m_last_mouse_pos.y(); }

  void setImage(const cv::Mat1f& im, double* min_val = 0, double* max_val = 0);
  void setImage(const cv::Mat1b& im);
  void setImage(const cv::Mat3b& im);

  void setRects(const std::list<cv::Rect>& rects);

signals:
  void mouseMoved(int image_x, int image_y);

protected:
  virtual void mouseMoveEvent( QMouseEvent * event );
  virtual void paintEvent(QPaintEvent * event);

private:
  QPoint m_last_mouse_pos;
  QImage m_image;
  std::list<cv::Rect> m_rects;
};

} // ntk

#endif // NTK_GUI_IMAGEWIDGET_H
