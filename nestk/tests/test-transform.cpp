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

#include <ntk/ntk.h>
#include <ntk/geometry/similarity_transform.h>

#include <QTransform>

using namespace ntk;

int main()
{
  ntk::ntk_debug_level = 1;

  cv::Point2f p (50,10);
  QPointF qt_p (p.x,p.y);

  QTransform qt_transform;
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.scale(5,5);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.translate(10,-1);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.rotate(1.0);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.scale(1/5.,1/5.);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.translate(-1,1);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.scale(7,7);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.rotate(-2.0);
  ntk_dbg_print(qt_transform.map(qt_p), 1);

  SimilarityTransform transform;
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 0, 5);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(10,-1), 0, 1);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 1, 1);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 0, 1/5.);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(-1,1), 0, 1);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 0, 7);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), -2, 1);
  ntk_dbg_print(transform.transform(p), 1);

  return 0;
}
