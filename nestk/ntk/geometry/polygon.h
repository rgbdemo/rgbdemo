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

#ifndef NTK_GEOMETRY_POLYGON_H
#define NTK_GEOMETRY_POLYGON_H

#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/utils/opencv_utils.h>

#ifdef USE_QT
# include <QPolygonF>
#endif

namespace ntk
{

class Pose3D;
class Polygon2d;
cv::Rect_<float> bounding_box(const Polygon2d& polygon);

class Polygon2d
{
public:
    Polygon2d() : points(1) {}

    cv::Rect boundingBox() { return bounding_box(*this) ;}
    int numVertices() const;
    void addNewSheet() { points.push_back(std::vector<cv::Point2f>()); }

    std::vector<cv::Point2f>& lastSheet()
    {
      ntk_assert(points.size() > 0, "Empty polygon");
      return points[points.size()-1];
    }

public:
    std::vector< std::vector<cv::Point2f> > points;
};

} // ntk

namespace ntk
{

ntk::Polygon2d project_bounding_box_to_image(const Pose3D& pose, const ntk::Rect3f& box);

cv::Rect_<float> bounding_box(const ntk::Polygon2d& polygon);
ntk::Rect3f bounding_box(const std::vector<cv::Point3f>& points);

#ifdef USE_QT
inline QPolygonF toQt(const ntk::Polygon2d& polygon)
{
  QPolygonF output;
  for (int i = 0; i < 4; ++i)
    output << QPointF(polygon.points[0][i].x, polygon.points[0][i].y);
  output << output[0];
  return output;
}
#endif

ntk::Polygon2d toPolygon(const cv::Rect_<float>& bbox);

} // ntk

#endif // NTK_GEOMETRY_POLYGON_H
