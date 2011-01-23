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

#include "image_widget.h"

#include <ntk/ntk.h>

#include <QMouseEvent>
#include <QPainter>

using namespace cv;

namespace ntk
{

void ImageWidget :: mouseMoveEvent ( QMouseEvent * event )
{
  if (m_image.isNull())
  {
    event->ignore();
    return;
  }

  int x = event->x() * (m_image.width() / float(width()));
  int y = event->y() * (m_image.height() / float(height()));
  m_last_mouse_pos = QPoint(x,y);
  emit mouseMoved(x,y);
}

void ImageWidget :: setRects(const std::list<cv::Rect>& rects)
{
  m_rects = rects;
  update();
}

void ImageWidget :: setImage(const cv::Mat1b& im)
{
  if (m_image.width() != im.cols
      || m_image.height() != im.rows)
    m_image = QImage(im.cols, im.rows, QImage::Format_RGB32);

  for (int r = 0; r < im.rows; ++r)
  {
    QRgb* ptr = (QRgb*) m_image.scanLine(r);
    for (int c = 0; c < im.cols; ++c)
    {
      int v = im(r,c);
      *ptr = qRgb(v,v,v);
      ++ptr;
    }
  }
  update();
}

void ImageWidget :: setImage(const cv::Mat1f& im, double* i_min_val, double* i_max_val)
{
  if (m_image.width() != im.cols
      || m_image.height() != im.rows)
    m_image = QImage(im.cols, im.rows, QImage::Format_RGB32);

  double min_val, max_val;
  if (i_min_val && i_max_val)
  {
    min_val = *i_min_val;
    max_val = *i_max_val;
  }
  else
    minMaxLoc(im, &min_val, &max_val);
  if (min_val == max_val)
    return;

  for (int r = 0; r < im.rows; ++r)
  {
    QRgb* ptr = (QRgb*) m_image.scanLine(r);
    const float* cv_ptr = im.ptr<float>(r);
    for (int c = 0; c < im.cols; ++c)
    {
      int v = 255*(*cv_ptr-min_val)/(max_val-min_val);
      v = ntk::saturate_to_range(v, 0, 255);
      int rgb = (0xff << 24) + (v << 16) + (v << 8) + v;
      *ptr = rgb;
      ++ptr;
      ++cv_ptr;
    }
  }
  update();
}

void ImageWidget :: setImage(const cv::Mat3b& im)
{
  if (m_image.width() != im.cols
      || m_image.height() != im.rows)
    m_image = QImage(im.cols, im.rows, QImage::Format_RGB32);

  for (int r = 0; r < im.rows; ++r)
  {
    QRgb* ptr = (QRgb*) m_image.scanLine(r);
    const uchar* cv_ptr = im.ptr(r);
    for (int i = 0; i < im.cols; ++i)
    {
      int rgb = 0xff << 24;
      rgb |= (*cv_ptr++);
      rgb |= ((*cv_ptr++) << 8);
      rgb |= ((*cv_ptr++) << 16);
      *ptr++ = rgb;
    }
  }
  update();
}

double ImageWidget :: scaleX() const
{
  return double(rect().width())/m_image.rect().width();
}

double ImageWidget :: scaleY() const
{
  return double(rect().height())/m_image.rect().height();
}

void ImageWidget :: paintEvent(QPaintEvent * event)
{
  double sx = scaleX();
  double sy = scaleY();

  QPen pen(Qt::yellow);
  pen.setWidth(2);

  QPainter painter(this);
  painter.drawImage(rect(), m_image, m_image.rect());

  painter.setPen(pen);
  foreach_const_it(it, m_rects, std::list<cv::Rect>)
  {
    const cv::Rect& r = *it;
    QRect qr (r.x*sx, r.y*sy, r.width*sx, r.height*sy);
    painter.drawRect(qr);
  }
}

} // ntk
