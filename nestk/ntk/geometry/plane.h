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

#ifndef NTK_GEOMETRY_PLANE_H
#define NTK_GEOMETRY_PLANE_H

#include <ntk/core.h>

namespace ntk
{

class Plane
{
public:
  Plane(double a, double b, double c, double d)
   : a(a), b(b), c(c), d(d)
  {}

  /*!
   * Construct a plane from a normal vector and a point.
   */
  Plane(const cv::Vec3f& normal, const cv::Point3f p);

  Plane() : a(0), b(0), c(0), d(0)
  {}

  cv::Vec3f normal() const;

  void set (double a_, double b_, double c_, double d_)
  { a = a_; b = b_; c = c_; d = d_; }

  cv::Point3f intersectionWithLine (const cv::Point3f& p1, const cv::Point3f& p2) const;

  double a,b,c,d;
};

void orthogonal_basis(cv::Vec3f& v1, cv::Vec3f& v2, const cv::Vec3f& v0);

void read_from_yaml(cv::FileNode node, Plane& p);
void write_to_yaml(cv::FileStorage& output_file, const std::string& name, const Plane& p);

} // ntk

#endif // NTK_GEOMETRY_PLANE_H
