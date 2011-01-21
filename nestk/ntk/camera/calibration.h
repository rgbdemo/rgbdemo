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
 * Author: Jorge Garcia Bueno <jgarcia@ing.uc3m.es>, (C) 2010
 */

#ifndef NTK_STEREO_CALIBRATION_H
#define NTK_STEREO_CALIBRATION_H

#include <ntk/core.h>
#include <opencv/cv.h>
#include <ntk/camera/rgbd_image.h>

namespace ntk
{

class MeshGenerator;
class Pose3D;

struct RGBDCalibration
{
  RGBDCalibration() :
    zero_rgb_distortion(true),
    zero_depth_distortion(true),
    depth_pose(0),
    rgb_pose(0),
    raw_rgb_size(640,480),
    rgb_size(480,480),
    raw_depth_size(204,204),
    depth_size(204,204),
    depth_baseline(7.5e-02),
    depth_offset(1090)
  {}

  ~RGBDCalibration();

  const cv::Size& rgbSize() const { return rgb_size; }
  void setRgbSize(cv::Size s) { rgb_size = s; }

  const cv::Size& rawRgbSize() const { return raw_rgb_size; }
  void setRawRgbSize(cv::Size s) { raw_rgb_size = s; }

  const cv::Size& depthSize() { return depth_size; }
  void loadFromFile(const char* filename);

  // Intrinsics of the color camera.
  cv::Mat1d rgb_intrinsics;
  cv::Mat1d rgb_distortion;
  bool zero_rgb_distortion;

  // Intrinsics of the depth camera.
  cv::Mat1d depth_intrinsics;
  cv::Mat1d depth_distortion;
  bool zero_depth_distortion;

  // Relative pose between camera. Depth is the reference.
  cv::Mat1d R,T;

  // Pose of the depth camera.
  Pose3D* depth_pose;

  // Pose of the color camera.
  Pose3D* rgb_pose;

  cv::Size raw_rgb_size;
  cv::Size rgb_size;

  cv::Size raw_depth_size;
  cv::Size depth_size;

  cv::Mat rgb_undistort_map1, rgb_undistort_map2;
  cv::Mat depth_undistort_map1, depth_undistort_map2;

  double depth_baseline;
  double depth_offset;
};

void crop_image(cv::Mat& image, cv::Size s);

void calibrationPattern(std::vector< std::vector<cv::Point3f> >& output,
                        int pattern_width,
                        int pattern_height,
                        float square_size,
                        int nb_images);

void calibrationCorners(const std::string& image_name,
                        const std::string& window_name,
                        int pattern_width, int pattern_height,
                        std::vector<cv::Point2f>& corners,
                        const cv::Mat& image,
                        float scale_factor);

void estimate_checkerboard_pose(const std::vector<cv::Point3f>& model,
                                const std::vector<cv::Point2f>& img_points,
                                const cv::Mat1d& calib_matrix,
                                cv::Mat1f& H);

} // ntk

#endif // NTK_STEREO_CALIBRATION_H
