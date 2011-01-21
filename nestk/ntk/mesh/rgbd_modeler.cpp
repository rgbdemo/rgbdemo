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

#include "rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>

using namespace cv;

namespace ntk
{

  void RGBDModeler :: addNewView(const RGBDImage& image, Pose3D& relative_pose)
  {
    Pose3D rgb_pose = *image.calibration()->rgb_pose;
    Pose3D depth_pose = *image.calibration()->depth_pose;
    depth_pose.applyTransformBefore(relative_pose);
    rgb_pose.applyTransformBefore(relative_pose);

    Pose3D world_to_camera_normal_pose;
    world_to_camera_normal_pose.applyTransformBefore(Vec3f(0,0,0), relative_pose.cvEulerRotation());
    Pose3D camera_to_world_normal_pose = world_to_camera_normal_pose; camera_to_world_normal_pose.invert();

    const Mat1f& depth_im = image.depth();

    for_all_rc(depth_im)
    {
      if (!image.depthMask()(r,c))
        continue;
      float depth = depth_im(r,c) + m_global_depth_offset;
      Point3f p3d = depth_pose.unprojectFromImage(Point2f(c,r), depth);
      Point3f p_rgb = rgb_pose.projectToImage(p3d);
      if (!is_yx_in_range(image.rgb(), p_rgb.y, p_rgb.x))
        continue;

      Vec3f camera_normal = image.normal()(r, c);
      Vec3f world_normal = camera_to_world_normal_pose.cameraTransform(camera_normal);
      normalize(world_normal);

      cv::Vec3b color = bgr_to_rgb(image.rgb()(p_rgb.y, p_rgb.x));
      m_mesh.vertices.push_back(p3d);
      m_mesh.colors.push_back(color);
      m_mesh.normals.push_back(world_normal);
    }
  }

} // ntk
