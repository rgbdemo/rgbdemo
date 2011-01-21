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

#include "mesh_generator.h"

#include <ntk/ntk.h>
#include <ntk/geometry/pose_3d.h>

using namespace cv;

namespace ntk
{

  MeshGenerator :: MeshGenerator()
    : m_use_color(false),
    m_mesh_type(PointCloudMesh),
    m_resolution_factor(1.0)
  {
  }

  void MeshGenerator :: setResolutionFactor(double f)
  {
    m_resolution_factor = f;
  }

  void MeshGenerator :: generatePointCloudMesh(const RGBDImage& image,
                                               const Pose3D& depth_pose,
                                               const Pose3D& rgb_pose)
  {
    m_mesh.clear();
    m_mesh.vertices.reserve(image.depth().rows*image.depth().cols);
    m_mesh.colors.reserve(image.depth().rows*image.depth().cols);
    m_mesh.normals.reserve(image.depth().rows*image.depth().cols);

    const cv::Mat1f& depth_im = image.depth();
    const cv::Mat1b& mask_im = image.depthMask();
    cv::Mat3f voxels (depth_im.size());
    cv::Mat3f rgb_points (depth_im.size());

    cv::Mat1b subsample_mask(mask_im.size());
    subsample_mask = 0;
    for (float r = 0; r < subsample_mask.rows-1; r += 1.0/m_resolution_factor)
      for (float c = 0; c < subsample_mask.cols-1; c += 1.0/m_resolution_factor)
        subsample_mask(ntk::math::rnd(r),ntk::math::rnd(c)) = 1;
    subsample_mask = mask_im & subsample_mask;

    depth_pose.unprojectFromImage(depth_im, subsample_mask, voxels);
    if (m_use_color)
      rgb_pose.projectToImage(voxels, subsample_mask, rgb_points);

    for (int r = 0; r < voxels.rows; ++r)
    {
      Vec3f* voxels_data = voxels.ptr<Vec3f>(r);
      const uchar* mask_data = subsample_mask.ptr<uchar>(r);
      for (int c = 0; c < voxels.cols; ++c)
      {
        if (!mask_data[c])
          continue;

        Vec3b color (0,0,0);
        if (m_use_color)
        {
          Point3f prgb = rgb_points(r,c);
          int i_y = ntk::math::rnd(prgb.y);
          int i_x = ntk::math::rnd(prgb.x);
          if (is_yx_in_range(image.rgb(), i_y, i_x))
          {
            Vec3b bgr = image.rgb()(i_y, i_x);
            color = Vec3b(bgr[2], bgr[1], bgr[0]);
          }
        }
        else
        {
          int g = 0;
          if (image.intensity().data)
            g = image.intensity()(r,c);
          else
            g = 255 * voxels_data[c][2] / 10.0;
          color = Vec3b(g,g,g);
        }

        m_mesh.vertices.push_back(voxels_data[c]);
        m_mesh.colors.push_back(color);
      }
    }
  }

  void MeshGenerator :: generateSurfelsMesh(const RGBDImage& image,
                                            const Pose3D& depth_pose,
                                            const Pose3D& rgb_pose)
  {
    double min_val = 0, max_val = 0;
    if (image.amplitude().data)
      minMaxLoc(image.amplitude(), &min_val, &max_val);

    std::vector<Surfel> surfels;
    m_mesh.clear();

    const cv::Mat1f& depth_im = image.depth();
    const cv::Mat1b& mask_im = image.depthMask();

    for_all_rc(depth_im)
    {
      int i_r = r;
      int i_c = c;
      if (!is_yx_in_range(depth_im, i_r, i_c))
        continue;

      if (!mask_im(r,c))
        continue;

      double depth = depth_im(i_r,i_c);
      cv::Point3f p = depth_pose.unprojectFromImage(Point2f(c,r), depth);

      Point3f normal = image.normal().data ? image.normal()(i_r, i_c) : Vec3f(0,0,1);

      Vec3b color (0,0,0);
      if (m_use_color)
      {
        cv::Point3f prgb = rgb_pose.projectToImage(p);
        int i_y = ntk::math::rnd(prgb.y);
        int i_x = ntk::math::rnd(prgb.x);
        if (is_yx_in_range(image.rgb(), i_y, i_x))
        {
          Vec3b bgr = image.rgb()(i_y, i_x);
          color = Vec3b(bgr[2], bgr[1], bgr[0]);
        }
      }
      else
      {
        int g = 0;
        if (image.amplitude().data)
          g = 255.0 * (image.amplitude()(i_r,i_c) - min_val) / (max_val-min_val);
        else
          g = 255 * depth / 10.0;
        color = Vec3b(g,g,g);
      }

      Surfel s;
      s.color = color;
      s.confidence = 0;
      s.location = p;
      s.normal = normal;
      s.n_views = 1;
      double normal_z = std::max(normal.z, 0.5f);
      s.radius = m_resolution_factor * ntk::math::sqrt1_2 * depth
          / (depth_pose.focalX() * normal_z);
      surfels.push_back(s);
    }

    m_mesh.buildFromSurfels(surfels, 1);
  }

  void MeshGenerator :: generate(const RGBDImage& image,
                                 const Pose3D& depth_pose,
                                 const Pose3D& rgb_pose)
  {;
    const Pose3D& real_depth_pose = depth_pose.isValid() ? depth_pose : *(image.calibration()->depth_pose);
    const Pose3D& real_rgb_pose = rgb_pose.isValid() ? rgb_pose : *(image.calibration()->rgb_pose);

    switch (m_mesh_type)
    {
    case TriangleMesh:
      return generateTriangleMesh(image, real_depth_pose, real_rgb_pose);
    case  PointCloudMesh:
      return generatePointCloudMesh(image, real_depth_pose, real_rgb_pose);
    case SurfelsMesh:
      return generateSurfelsMesh(image, real_depth_pose, real_rgb_pose);
    }
  }

  void MeshGenerator :: generateTriangleMesh(const RGBDImage& image,
                                             const Pose3D& depth_pose,
                                             const Pose3D& rgb_pose)
  {
    const double max_delta_depth = 0.05;
    const Mat1f& depth_im = image.depth();
    const Mat1b& mask_im = image.depthMask();
    m_mesh.clear();
    if (m_use_color)
      image.rgb().copyTo(m_mesh.texture);
    else if (image.intensity().data)
      toMat3b(normalize_toMat1b(image.intensity())).copyTo(m_mesh.texture);
    else
    {
      m_mesh.texture.create(depth_im.size());
      m_mesh.texture = Vec3b(255,255,255);
    }
    m_mesh.vertices.reserve(depth_im.cols*depth_im.rows);
    m_mesh.texcoords.reserve(depth_im.cols*depth_im.rows);
    m_mesh.colors.reserve(depth_im.cols*depth_im.rows);
    Mat1i vertice_map(depth_im.size());
    vertice_map = -1;
    for_all_rc(depth_im)
    {
      if (!mask_im(r,c))
        continue;
      double depth = depth_im(r,c);
      Point3f p3d = depth_pose.unprojectFromImage(Point3f(c,r,depth));
      Point3f p2d_rgb;
      Point2f texcoords;
      if (m_use_color)
      {
        p2d_rgb = rgb_pose.projectToImage(p3d);
        texcoords = Point2f(p2d_rgb.x/image.rgb().cols, p2d_rgb.y/image.rgb().rows);
      }
      else
      {
        p2d_rgb = Point3f(c,r,depth);
        texcoords = Point2f(p2d_rgb.x/image.intensity().cols, p2d_rgb.y/image.intensity().rows);
      }
      vertice_map(r,c) = m_mesh.vertices.size();
      m_mesh.vertices.push_back(p3d);
      // m_mesh.colors.push_back(bgr_to_rgb(im.rgb()(p2d_rgb.y, p2d_rgb.x)));
      m_mesh.texcoords.push_back(texcoords);
    }

    for_all_rc(vertice_map)
    {
      if (vertice_map(r,c) < 0)
        continue;

      if ((c < vertice_map.cols - 1) &&  (r < vertice_map.rows - 1) &&
          (vertice_map(r+1,c)>=0) && (vertice_map(r,c+1) >= 0) &&
          (std::abs(depth_im(r,c) - depth_im(r+1, c)) < max_delta_depth) &&
          (std::abs(depth_im(r,c) - depth_im(r, c+1)) < max_delta_depth))
      {
        Face f;
        f.indices[2] = vertice_map(r,c);
        f.indices[1] = vertice_map(r,c+1);
        f.indices[0] = vertice_map(r+1,c);
        m_mesh.faces.push_back(f);
      }

      if ((c > 0) &&  (r < vertice_map.rows - 1) &&
          (vertice_map(r+1,c)>=0) && (vertice_map(r+1,c-1) >= 0) &&
          (std::abs(depth_im(r,c) - depth_im(r+1, c)) < max_delta_depth) &&
          (std::abs(depth_im(r,c) - depth_im(r+1, c-1)) < max_delta_depth))
      {
        Face f;
        f.indices[2] = vertice_map(r,c);
        f.indices[1] = vertice_map(r+1,c);
        f.indices[0] = vertice_map(r+1,c-1);
        m_mesh.faces.push_back(f);
      }
    }
  }

} // ntk
