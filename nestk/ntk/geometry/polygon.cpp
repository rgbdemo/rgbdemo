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

#include "polygon.h"
#include <ntk/numeric/utils.h>
#include <ntk/geometry/pose_3d.h>

using namespace cv;

namespace ntk
{

int Polygon2d :: numVertices() const
{
  int n = 0;
  foreach_idx(i, points)
      n += points[i].size();
  return n;
}

cv::Rect_<float> bounding_box(const Polygon2d& polygon)
{
  if (polygon.numVertices() == 0)
    return cv::Rect_<float>();

  double min_x = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();

  for (unsigned s = 0; s < polygon.points.size(); ++s)
    for (unsigned p = 0; p < polygon.points[s].size(); ++p)
    {
      double x = polygon.points[s][p].x;
      double y = polygon.points[s][p].y;
      min_x = ntk::math::min(min_x, x);
      max_x = ntk::math::max(max_x, x);
      min_y = ntk::math::min(min_y, y);
      max_y = ntk::math::max(max_y, y);
    }

  return cv::Rect_<float>(min_x, min_y, max_x-min_x, max_y-min_y);
}


ntk::Polygon2d project_bounding_box_to_image(const Pose3D& pose, const ntk::Rect3f& box)
{
  const int links[] = { 0, 1, 3, 2, 0, -1,
                        0, 4, 6, 2, 0, -1,
                        4, 5, 7, 6, 4, -1,
                        0, 1, 5, 4, 0, -1,
                        1, 3, 7, 5, 1, -1,
                        2, 3, 7, 6, 2 };

  Point3f cube_points[8];
  Point3f proj_cube_points[8];

  const double xvals [] = {box.x, box.x+box.width};
  const double yvals [] = {box.y, box.y+box.height};
  const double zvals [] = {box.z, box.z+box.depth};

  int index = 0;
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k)
      {
        Point3f p(xvals[i], yvals[j], zvals[k]);
        cube_points[index] = p;
        Point3f im_p = pose.projectToImage(p);
        proj_cube_points[index] = im_p;
        ++index;
      }

  ntk::Polygon2d polygon;
  for (int i = 0; i < sizeof(links)/sizeof(int); ++i)
  {
    if (links[i] < 0)
    {
      polygon.addNewSheet();
      continue;
    }

    const Point3f& p = proj_cube_points[links[i]];
    polygon.lastSheet().push_back(Point2f(p.x, p.y));
  }
  return polygon;
}

ntk::Polygon2d toPolygon(const cv::Rect_<float>& bbox)
{
  ntk::Polygon2d polygon;
  polygon.points[0].push_back(Point2f(bbox.x, bbox.y));
  polygon.points[0].push_back(Point2f(bbox.x+bbox.width, bbox.y));
  polygon.points[0].push_back(Point2f(bbox.x+bbox.width, bbox.y+bbox.height));
  polygon.points[0].push_back(Point2f(bbox.x, bbox.y+bbox.height));
  return polygon;
}

} // ntk
