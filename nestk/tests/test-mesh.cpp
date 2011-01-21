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

#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/geometry/pose_3d.h>

#include <opencv/highgui.h>

#include <QApplication>

using namespace ntk;
using namespace cv;

int main(int argc, char** argv)
{
  QApplication app(argc,argv); // renderer uses QT
  Mesh mesh;
  mesh.loadFromPlyFile("mesh.ply");
  mesh.saveToPlyFile("output.ply");

#if 0
  Pose3D pose; pose.parseFromBundler("mesh.pose");

  double tx,ty,tz,rx,ry,rz,field_of_view;
  pose.toBlenderParameters(800, 600, &tx,&ty,&tz,&rx,&ry,&rz,&field_of_view);
  ntk_dbg_print(tx,0);
  ntk_dbg_print(ty,0);
  ntk_dbg_print(tz,0);
  ntk_dbg_print(rx,0);
  ntk_dbg_print(ry,0);
  ntk_dbg_print(rz,0);
  ntk_dbg_print(field_of_view,0);
#endif

#if 1
  Pose3D pose_from_blender;
  pose_from_blender.parseBlenderFile("mesh15.pose.blender", 800, 600);
#if 0
  pose_from_blender.loadFromBlenderParameters(
      1.57, 2.52, -1, /* object tx ty tz from blender */
      84, 44, 142, /* object rx ry rz from blender in degrees */
      73.8, /* camera field of view in degrees (D flags in camera settings) */
      800, 600); /* image size */
  ntk_dbg_print(pose_from_blender.focalLength(), 0);
#endif
#endif

  MeshRenderer renderer(mesh, 800, 600);

  cv::Mat4b image (Size(800,600));
  cv::Mat3b color_image;
  cv::Mat1f depth;

#if 0
  renderer.renderToImage(image, pose, 0);
  color_image = renderer.colorBuffer();
  imwrite("color.png", color_image);
  depth = renderer.depthBuffer();
  normalize(depth, depth, 0, 255, cv::NORM_MINMAX);
  imwrite("depth.png", Mat1b(depth));
#endif

#if 1
  renderer.renderToImage(image, pose_from_blender, 0);
  color_image = renderer.colorBuffer();
  imwrite("color2.png", color_image);
  depth = renderer.depthBuffer();
  normalize(depth, depth, 0, 255, cv::NORM_MINMAX);
  imwrite("depth2.png", Mat1b(depth));
#endif

  ntk_dbg_print(mesh.vertices.size(), 0);
  ntk_dbg_print(mesh.faces.size(), 0);
  ntk_dbg_print(mesh.colors.size(), 0);
  return 0;
}
