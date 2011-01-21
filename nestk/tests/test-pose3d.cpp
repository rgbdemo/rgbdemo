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

#include <ntk/geometry/pose_3d.h>
#include <ntk/utils/opencv_utils.h>

using namespace ntk;
using namespace cv;

void original_code()
{
#if 0
  Pose3D p;
  p.setCameraParameters(500,400,322,235);
  p.applyTransformBefore(Vec3f(-4, 3, 0.1), vgl_rotation_3d<double>(-0.1, 0.2, 0.3));
  p.applyTransformAfter(Vec3f(4, 4, 0.2), vgl_rotation_3d<double>(0.5, 0.2, 0.3));
  p.applyTransformBefore(Vec3f(2, 3, -0.1), vgl_rotation_3d<double>(0.1, -0.1, 0.3));
  p.applyTransformBefore(Vec3f(4, -0.2, 0.3), vgl_rotation_3d<double>(-0.1, 0.5, 0.3));
  p.applyTransformAfter(Vec3f(8, 3, 0.4), vgl_rotation_3d<double>(0.4, 0.2, -0.3));

  Point3f p1 (3.4, 5.8, 4.2);
  Point3f p2 (-0.3, -5.8, 0.1);

  Point3f p1b = p.projectToImage(p1);
  Point3f p2b = p.projectToImage(p2);

  Point3f p3b = p.unprojectFromImage(Point2f(200, 250), 1.2);
  Point3f p4b = p.unprojectFromImage(Point2f(0, 0), 0.5);

  ntk_dbg_print(p1b, 0);
  ntk_dbg_print(p2b, 0);
  ntk_dbg_print(p3b, 0);
  ntk_dbg_print(p4b, 0);

  p.invert();
  Point3f p5b = p.projectToImage(p1);
  Point3f p6b = p.projectToImage(p2);

  Point3f p7b = p.unprojectFromImage(Point2f(150, 200), 1.5);
  Point3f p8b = p.unprojectFromImage(Point2f(0, 1), 0.8);
  ntk_dbg_print(p5b, 0);
  ntk_dbg_print(p6b, 0);
  ntk_dbg_print(p7b, 0);
  ntk_dbg_print(p8b, 0);
#endif

#if 0
  "p1b: [-174.24 613.15 -14.7736]"
  "p2b: [-2268.24 1239.1 -3.59793]"
  "p3b: [-17.5037 1.48109 -10.6162]"
  "p4b: [-17.1449 2.06466 -10.247]"
  "p5b: [-420.155 -11.5039 6.79462]"
  "p6b: [-1178.82 194.594 7.4783]"
  "p7b: [13.5603 11.1002 6.37541]"
  "p8b: [13.7224 10.9287 7.11546]"
#endif
}

void new_code()
{
  Pose3D p;
  p.setCameraParameters(500,400,322,235);
  p.applyTransformBefore(Vec3f(-4, 3, 0.1), Vec3f(-0.1, 0.2, 0.3));
  p.applyTransformAfter(Vec3f(4, 4, 0.2), Vec3f(0.5, 0.2, 0.3));
  p.applyTransformBefore(Vec3f(2, 3, -0.1), Vec3f(0.1, -0.1, 0.3));
  p.applyTransformBefore(Vec3f(4, -0.2, 0.3), Vec3f(-0.1, 0.5, 0.3));
  p.applyTransformAfter(Vec3f(8, 3, 0.4), Vec3f(0.4, 0.2, -0.3));

  Point3f p1 (3.4, 5.8, 4.2);
  Point3f p2 (-0.3, -5.8, 0.1);

  Point3f p1b = p.projectToImage(p1);
  Point3f p2b = p.projectToImage(p2);

  Point3f p3b = p.unprojectFromImage(Point2f(200, 250), 1.2);
  Point3f p4b = p.unprojectFromImage(Point2f(0, 0), 0.5);

  ntk_dbg_print(p1b, 0);
  ntk_dbg_print(p2b, 0);
  ntk_dbg_print(p3b, 0);
  ntk_dbg_print(p4b, 0);

  p.invert();
  Point3f p5b = p.projectToImage(p1);
  Point3f p6b = p.projectToImage(p2);

  Point3f p7b = p.unprojectFromImage(Point2f(150, 200), 1.5);
  Point3f p8b = p.unprojectFromImage(Point2f(0, 1), 0.8);
  ntk_dbg_print(p5b, 0);
  ntk_dbg_print(p6b, 0);
  ntk_dbg_print(p7b, 0);
  ntk_dbg_print(p8b, 0);
}

void simple_code()
{
  Pose3D p;
  p.setCameraParameters(200,200,100,100);
  ntk_dbg_print(p.projectToImage(Point3f(1,1,-2)), 0);
  // 200,0,2

  p.applyTransformBefore(Vec3f(1,0,0), Vec3f(0,0,M_PI/2));
  p.applyTransformBefore(Vec3f(0.5,1,0.2), Vec3f(0.1,0.2,0.3));
  ntk_dbg_print(p.cameraTransform(Point3f(1,1,-2)), 0);
}

void euler_code()
{
  {
    Pose3D p;
    p.setCameraParameters(200,200,100,100);
    p.applyTransformBefore(Vec3f(0,0,0), Vec3f(0,0,3.14));
    ntk_dbg_print(p, 0);
    ntk_dbg_print(p.cameraTransform(Point3f(1,2,3)), 0);
  }

  Pose3D p;
  p.setCameraParameters(200,200,100,100);
  p.applyTransformBefore(Vec3f(1,2,3), Vec3f(1,2,0.5));
  p.applyTransformBefore(Vec3f(2,3,4), Vec3f(2,-1,0.8));

  Pose3D p2;
  p2.setCameraParameters(200,200,100,100);
  p2.applyTransformBefore(p.cvTranslation(), p.cvEulerRotation());

  ntk_dbg_print(p, 0);
  ntk_dbg_print(p2, 0);

  ntk_dbg_print(p.cameraTransform(Point3f(-5, 8, 9)), 0);
  ntk_dbg_print(p2.cameraTransform(Point3f(-5, 8, 9)), 0);
}

int main()
{  
  euler_code();
  original_code();
  simple_code();
  new_code();
}
