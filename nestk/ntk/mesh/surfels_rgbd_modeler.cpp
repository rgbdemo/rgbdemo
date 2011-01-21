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

#include "surfels_rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>

using namespace cv;

namespace ntk
{

  void SurfelsRGBDModeler :: addNewView(const RGBDImage& image, Pose3D& relative_pose)
  {
    const float update_max_normal_angle = 60;
    const float update_max_dist = 0.1;

    Pose3D rgb_pose = *image.calibration()->rgb_pose;
    Pose3D depth_pose = *image.calibration()->depth_pose;
    depth_pose.applyTransformBefore(relative_pose);
    rgb_pose.applyTransformBefore(relative_pose);

    Pose3D world_to_camera_normal_pose;
    world_to_camera_normal_pose.applyTransformBefore(relative_pose);
    Pose3D camera_to_world_normal_pose = world_to_camera_normal_pose; camera_to_world_normal_pose.invert();

    const Mat1f& depth_im = image.depth();
    Mat1b covered_pixels (depth_im.size());
    covered_pixels = 0;

    // Surfel updating.
    foreach_idx(i, m_surfels)
    {
      Surfel& surfel = m_surfels[i];
      if (!surfel.enabled())
        continue;
      Point3f surfel_2d = depth_pose.projectToImage(surfel.location);
      bool surfel_deleted = false;
      int r = ntk::math::rnd(surfel_2d.y);
      int c = ntk::math::rnd(surfel_2d.x);
      int d = ntk::math::rnd(surfel_2d.z);
      if (!is_yx_in_range(depth_im, r, c) || !image.depthMask()(r, c))
        continue;

      Vec3f camera_normal = image.normal()(r, c);
      Vec3f world_normal = camera_to_world_normal_pose.cameraTransform(camera_normal);
      normalize(world_normal);

      Vec3f eyev = camera_eye_vector(depth_pose, r, c);
      double camera_angle = acos(camera_normal.dot(eyev));

      float normal_angle = acos(world_normal.dot(surfel.normal));
      if (normal_angle > (update_max_normal_angle*M_PI/180.0))
      {
        // Removal check. If a surfel has a different normal and is closer to the camera
        // than the new scan, remove it.
        if (surfel_2d.z > depth_im(r,c) && surfel.n_views < 2)
        {
          surfel = Surfel();
          surfel_deleted = true;
        }
        else
          covered_pixels(r,c) = 1;
        continue;
      }

      // If existing surfel is far from new depth value:
      // - If existing one had a worst point of view, and was seen only once, remove it.
      // - Otherwise do not include the new one.
      if (std::abs(surfel_2d.z - depth_im(r,c)) > update_max_dist)
      {
        if (surfel.min_camera_angle > camera_angle && surfel.n_views < 2)
        {
          surfel = Surfel(); // remove the old one.
          surfel_deleted = true;
        }
        else
          covered_pixels(r,c) = 1;
        continue;
      }

      // Compatible surfel found.
      const float depth = depth_im(r,c) + m_global_depth_offset;

      Point3f p3d = depth_pose.unprojectFromImage(Point2f(c,r), depth);
      Point3f p_rgb = rgb_pose.projectToImage(p3d);
      if (!is_yx_in_range(image.rgb(), p_rgb.y, p_rgb.x))
        continue;

      cv::Vec3b rgb_color = bgr_to_rgb(image.rgb()(p_rgb.y, p_rgb.x));

      surfel.location = (surfel.location*float(surfel.n_views) + p3d) * (1.0f/(surfel.n_views+1));
      surfel.normal = (surfel.normal*float(surfel.n_views) + Point3f(world_normal)) * (1.0f/(surfel.n_views+1));
      surfel.color = (Vec3f(surfel.color)*float(surfel.n_views) + Vec3f(rgb_color)) * (1.f/(surfel.n_views+1));
      if (camera_angle < surfel.min_camera_angle)
      {
        surfel.normal = world_normal;
        surfel.min_camera_angle = camera_angle;
      }
      surfel.radius = std::min((double)surfel.radius,
                               1.0 * ntk::math::sqrt1_2 * depth
                               / (depth_pose.focalX() * camera_normal[2]));
      surfel.n_views += 1;
      covered_pixels(r,c) = 1;
    }

    imwrite_normalized("debug_covered.png", covered_pixels);
    imwrite_normalized("debug_depth_mask.png", image.depthMask());

    // Surfel addition
    for (int r = 0; r < depth_im.rows; r += 1)
    for (int c = 0; c < depth_im.cols; c += 1)
    {
      if (!image.depthMask()(r,c) || covered_pixels(r,c))
        continue;
      float depth = depth_im(r,c) + m_global_depth_offset;
      Point3f p3d = depth_pose.unprojectFromImage(Point2f(c,r), depth);
      Point3f p_rgb = rgb_pose.projectToImage(p3d);
      if (!is_yx_in_range(image.rgb(), p_rgb.y, p_rgb.x))
        continue;
      cv::Vec3b rgb_color = bgr_to_rgb(image.rgb()(p_rgb.y, p_rgb.x));

      Vec3f camera_normal = image.normal()(r, c);
      Vec3f world_normal = camera_to_world_normal_pose.cameraTransform(camera_normal);
      normalize(world_normal);

      Vec3f eyev = camera_eye_vector(depth_pose, r, c);
      double camera_angle = acos(camera_normal.dot(eyev));

      Surfel surfel;
      surfel.location = p3d;
      surfel.normal = world_normal;
      surfel.color = rgb_color;
      // int b = 255.*(camera_angle*180./M_PI)/90.;
      // surfel.color = Vec3b(b,b,b);
      surfel.min_camera_angle = camera_angle;
#if 0
      ntk_dbg_print(eyev, 1);
      ntk_dbg_print(camera_normal, 1);
      ntk_dbg_print(camera_angle*180./M_PI, 1);
      ntk_dbg_print(b, 1);
#endif
      // surfel.radius = ntk::math::sqrt1_2 * depth / (depth_pose.focalX() * camera_normal[2]);
      double camera_normal_z = std::max(camera_normal[2], 0.3f);
      surfel.radius = 2 * ntk::math::sqrt1_2 * depth / (depth_pose.focalX() * camera_normal_z);
      surfel.n_views = 1;
      m_surfels.push_back(surfel);
    }

    ntk_dbg_print(m_surfels.size(), 1);
  }

  void ICPSurfelsRGBDModeler :: addNewView(const RGBDImage& image, Pose3D& relative_pose)
  {
    Pose3D corrected_relative_pose = fixRelativePose(image, relative_pose);
    SurfelsRGBDModeler::addNewView(image, corrected_relative_pose);
    relative_pose = corrected_relative_pose;
  }

  void icpIteration(Pose3D& delta_pose, Mesh& ref_cloud, Mesh& new_cloud);

  Pose3D ICPSurfelsRGBDModeler :: fixRelativePose(const RGBDImage& image,
                                                  const Pose3D& relative_pose)
  {
    Pose3D rgb_pose = *image.calibration()->rgb_pose;
    Pose3D depth_pose = *image.calibration()->depth_pose;
    Pose3D current_relative_pose = relative_pose;
    Mesh cloud;
    const Mat1f& depth_im = image.depth();

    static int global_iteration = -1;
    global_iteration++;

    for (int iteration = 0; iteration < 5; ++iteration)
    {
      Pose3D rgb_pose = *image.calibration()->rgb_pose;
      Pose3D depth_pose = *image.calibration()->depth_pose;
      depth_pose.applyTransformBefore(current_relative_pose);
      rgb_pose.applyTransformBefore(current_relative_pose);

      Pose3D world_to_camera_normal_pose;
      world_to_camera_normal_pose.applyTransformBefore(Vec3f(0,0,0), relative_pose.cvEulerRotation());
      Pose3D camera_to_world_normal_pose = world_to_camera_normal_pose; camera_to_world_normal_pose.invert();

      // Compute new partial mesh
      cloud.clear();
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
        cloud.vertices.push_back(p3d);
        cloud.colors.push_back(color);
        cloud.normals.push_back(world_normal);
      }

      if (m_point_cloud.vertices.size() == 0)
        break;

      Pose3D delta_pose;
      icpIteration(delta_pose, m_point_cloud, cloud);
      delta_pose.invert();
      current_relative_pose.applyTransformAfter(delta_pose);
      // cloud.saveToPlyFile(cv::format("mesh%04d-%d.ply", global_iteration, iteration).c_str());
      // m_point_cloud.saveToPlyFile(cv::format("ref_mesh%04d-%d.ply", global_iteration, iteration).c_str());
    } // iteration

    //  cloud.saveToPlyFile(cv::format("mesh%04d.ply", global_iteration).c_str());

    m_point_cloud.vertices.insert(m_point_cloud.vertices.end(), stl_bounds(cloud.vertices));
    m_point_cloud.normals.insert(m_point_cloud.normals.end(), stl_bounds(cloud.normals));
    m_point_cloud.colors.insert(m_point_cloud.colors.end(), stl_bounds(cloud.colors));
    return current_relative_pose;
  }

  cv::RNG rgen;

  void computeAssociations( std::vector< std::pair<int,int> >& associations,
                            Mesh& ref_cloud,
                            Mesh& new_cloud)
  {
    const double max_dist = 0.01;
    const double max_normal_angle = 40;
    for (int i = 0; i < new_cloud.vertices.size(); i += new_cloud.vertices.size()/1000 + 1)
    {
      Point3f pnew = new_cloud.vertices[i];
      int best_j = -1;
      double best_dist = FLT_MAX;
      foreach_idx(j, ref_cloud.vertices)
      {
        Point3f pref = ref_cloud.vertices[j];
        Point3f dp = pnew-pref;

        double dist = norm(dp);
        if (dist > max_dist || dist > best_dist)
          continue;

        if (acos(ref_cloud.normals[j].dot(new_cloud.normals[i])) > max_normal_angle)
          continue;

        best_dist = dist;
        best_j = j;
      }
      if (best_j < 0)
        continue;
      Vec3b color (i%255, i%255, i%255);
      //ref_cloud.colors[best_j] = color;
      //new_cloud.colors[i] = color;
      associations.push_back(std::make_pair(best_j, i));
    }
  }

  class icp_fitness : public ntk::CostFunction
  {
  public:
    icp_fitness(const std::vector< std::pair<int,int> >& associations,
                const Mesh& ref_cloud, const Mesh& new_cloud)
      : CostFunction(6, associations.size()*3),
      associations(associations),
      ref_cloud(ref_cloud),
      new_cloud(new_cloud)
    {

    }

    virtual void evaluate(const std::vector<double>& x, std::vector<double>& fx) const
    {
      Pose3D delta_pose;
      delta_pose.applyTransformBefore(Point3f(x[0],x[1],x[2]),
                                      Vec3f(x[3],x[4],x[5]));
      std::fill(stl_bounds(fx), 0);
      int fx_index = 0;
      foreach_idx(i, associations)
      {
        Point3f pref = ref_cloud.vertices[associations[i].first];
        Point3f pnew = new_cloud.vertices[associations[i].second];
        pnew = delta_pose.cameraTransform(pnew);
        const double error_scale = 1000;
        fx[fx_index++] = error_scale * (pref.x-pnew.x);
        fx[fx_index++] = error_scale * (pref.y-pnew.y);
        fx[fx_index++] = error_scale * (pref.z-pnew.z);
      }
    }

  public:
    const std::vector< std::pair<int,int> >& associations;
    const Mesh& ref_cloud;
    const Mesh& new_cloud;
  };

  void optimizePose(Pose3D& delta_pose,
                    const std::vector< std::pair<int,int> >& associations,
                    const Mesh& ref_cloud,
                    const Mesh& new_cloud)
  {
    std::vector<double> x(6, 0);
    icp_fitness f (associations, ref_cloud, new_cloud);
    LevenbergMarquartMinimizer minimizer;
    minimizer.minimize(f, x);
    minimizer.diagnoseOutcome();
    delta_pose.applyTransformBefore(Point3f(x[0],x[1],x[2]),
                                    Vec3f(x[3],x[4],x[5]));
    ntk_dbg_print(x[0], 1);
    ntk_dbg_print(x[1], 1);
    ntk_dbg_print(x[2], 1);
    ntk_dbg_print(x[3], 1);
    ntk_dbg_print(x[4], 1);
    ntk_dbg_print(x[5], 1);
  }

  void icpIteration(Pose3D& delta_pose, Mesh& ref_cloud, Mesh& new_cloud)
  {
    std::vector< std::pair<int,int> > associations;
    TimeCount tc_assoc("Compute Associations");
    computeAssociations(associations, ref_cloud, new_cloud);
    tc_assoc.stop();

    TimeCount tc_optimize("Optimize Pose");
    optimizePose(delta_pose, associations, ref_cloud, new_cloud);
    tc_optimize.stop();
  }

  void SurfelsRGBDModeler :: computeMesh()
  {
    m_mesh.buildFromSurfels(m_surfels, m_min_views);
  }

} // ntk
