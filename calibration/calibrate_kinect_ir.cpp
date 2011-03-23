/**
 * This file is part of the rgbdemo project.
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
 * Author: Nicolas Burrus <nicolas@burrus.name>, (C) 2010, 2011
 */

#include "calibration_common.h"

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <opencv/cv.h>
#include <fstream>

#include <QDir>
#include <QDebug>

using namespace ntk;
using namespace cv;

// example command line (for copy-n-paste):
// calibrate_one_camera -w 8 -h 6 -o camera.yml images

namespace global
{
ntk::arg<const char*> opt_image_directory("--input", "RGBD images directory", "grab1");
ntk::arg<const char*> opt_output_file("--output", "Output YAML filename", "kinect_calibration.yml");
ntk::arg<int> opt_pattern_width("--pattern-width", "Pattern width (number of inner squares)", 10);
ntk::arg<int> opt_pattern_height("--pattern-height", "Pattern height (number of inner squares)", 7);
ntk::arg<float> opt_square_size("--pattern-size", "Square size in used defined scale", 0.025);
ntk::arg<bool> opt_ignore_distortions("--no-depth-undistort", "Ignore distortions for depth images(faster processing)", false);

const cv::Size image_size(640,480);

QDir images_dir;
QStringList images_list;

cv::Mat1d depth_intrinsics;
cv::Mat1d depth_distortion;

cv::Mat1d rgb_intrinsics;
cv::Mat1d rgb_distortion;

// stereo transform.
cv::Mat1d R, T;

double depth_offset = 1090;
double depth_baseline = 7.5e-02;
}

struct DepthCalibrationPoint
{
  DepthCalibrationPoint(double kinect_raw, double estimated)
    : kinect_raw(kinect_raw),
      estimated(estimated)
  {}
  double kinect_raw;
  double estimated;
};

static void load_intensity_file(const std::string& dir, std::string& full_filename, cv::Mat3b& image)
{
  if (is_file(dir + "/raw/intensity.raw"))
  {
    full_filename = dir + "/raw/intensity.raw";
    cv::Mat1f fim = imread_Mat1f_raw(full_filename);
    ntk_ensure(fim.data, ("Could not read raw intensity image from " + full_filename).c_str());
    image = toMat3b(normalize_toMat1b(fim));
  }
  else if (is_file(dir + "/raw/intensity.yml"))
  {
    full_filename = dir + "/raw/intensity.yml";
    cv::Mat1f fim = imread_yml(full_filename);
    ntk_ensure(fim.data, ("Could not read raw intensity image from " + full_filename).c_str());
    image = toMat3b(normalize_toMat1b(fim));
  }
  else if (is_file(dir + "/raw/intensity.png"))
  {
    full_filename = dir + "/raw/intensity.png";
    image = imread(full_filename, 1);
    ntk_ensure(image.data, ("Could not read raw intensity image from " + full_filename).c_str());
  }
}

void calibrate_kinect_depth(std::vector< std::vector<Point2f> >& stereo_corners,
                            std::vector< DepthCalibrationPoint >& depth_values)
{
  std::vector< std::vector<Point2f> > good_corners;
  stereo_corners.resize(global::images_list.size());
  for (int i_image = 0; i_image < global::images_list.size(); ++i_image)
  {
    QString filename = global::images_list[i_image];
    QDir cur_image_dir (global::images_dir.absoluteFilePath(filename));

    std::string full_filename;
    cv::Mat3b image;
    load_intensity_file(cur_image_dir.path().toStdString(), full_filename, image);
    ntk_ensure(image.data, "Could not load color image");

    kinect_shift_ir_to_depth(image);

    std::vector<Point2f> current_view_corners;
    calibrationCorners(full_filename, "corners",
                       global::opt_pattern_width(), global::opt_pattern_height(),
                       current_view_corners, image, 1);

    if (current_view_corners.size() == global::opt_pattern_height()*global::opt_pattern_width())
    {
      good_corners.push_back(current_view_corners);
      stereo_corners[i_image] = current_view_corners;
      show_corners(image, current_view_corners, 1);
    }
    else
    {
      ntk_dbg(0) << "Warning: corners not detected";
      stereo_corners[i_image].resize(0);
    }
  }

  std::vector< std::vector<Point3f> > pattern_points;
  calibrationPattern(pattern_points,
                     global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                     good_corners.size());

  ntk_assert(pattern_points.size() == good_corners.size(), "Invalid points size");

  std::vector<Mat> rvecs, tvecs;

  int flags = 0;
  if (global::opt_ignore_distortions())
    flags = CV_CALIB_ZERO_TANGENT_DIST;

  double error = calibrateCamera(pattern_points, good_corners, global::image_size,
                                 global::depth_intrinsics, global::depth_distortion,
                                 rvecs, tvecs, flags);

  if (global::opt_ignore_distortions())
    global::depth_distortion = 0.f;

  ntk_dbg_print(error, 1);
  int good_i = 0;
  foreach_idx(stereo_i, stereo_corners)
  {
    if (stereo_corners[stereo_i].size() > 0)
    {
      QString filename = global::images_list[stereo_i];
      QDir cur_image_dir (global::images_dir.absoluteFilePath(filename));
      std::string full_filename;
      cv::Mat3b image;
      load_intensity_file(cur_image_dir.path().toStdString(), full_filename, image);
      ntk_ensure(image.data, "Could not load intensity image");
      kinect_shift_ir_to_depth(image);

      cv::Mat1f depth_image;

      if (is_file(cur_image_dir.absoluteFilePath("raw/depth.yml").toStdString()))
      {
        full_filename = cur_image_dir.absoluteFilePath("raw/depth.yml").toStdString();
        depth_image = imread_yml(full_filename);
      }
      else if (is_file(cur_image_dir.absoluteFilePath("raw/depth.raw").toStdString()))
      {
        full_filename = cur_image_dir.absoluteFilePath("raw/depth.raw").toStdString();
        depth_image = imread_Mat1f_raw(full_filename);
      }
      ntk_ensure(depth_image.data, "Could not load depth image");

      cv::Mat3b undistorted_image;
      if (global::opt_ignore_distortions())
        image.copyTo(undistorted_image);
      else
        undistort(image, undistorted_image, global::depth_intrinsics, global::depth_distortion);

      std::vector<Point2f> current_view_corners;
      calibrationCorners(full_filename, "corners",
                         global::opt_pattern_width(), global::opt_pattern_height(),
                         current_view_corners, undistorted_image, 1);

      if (current_view_corners.size() == (global::opt_pattern_width()*global::opt_pattern_height()))
      {
        stereo_corners[stereo_i] = current_view_corners;
        show_corners(undistorted_image, stereo_corners[stereo_i], 200);
      }
      else
      {
        stereo_corners[stereo_i].resize(0);
        continue;
      }

      // Generate calibration points
      {
        // FIXME: why rvecs and tvecs from calibrateCamera seems to be nonsense ?
        // calling findExtrinsics another time to get good chessboard estimations.

        cv::Mat1f H;
        estimate_checkerboard_pose(pattern_points[0],
                                   current_view_corners,
                                   global::depth_intrinsics,
                                   H);
        Pose3D pose;
        pose.setCameraParametersFromOpencv(global::depth_intrinsics);
        ntk_dbg_print(pose, 1);
        pose.setCameraTransform(H);

        foreach_idx(pattern_i, pattern_points[0])
        {
          ntk_dbg_print(pattern_points[0][pattern_i], 1);
          Point3f p = pose.projectToImage(pattern_points[0][pattern_i]);
          ntk_dbg_print(p, 1);
          double kinect_raw = depth_image(p.y, p.x);
          if (!(kinect_raw < 2047)) continue;
          ntk_dbg_print(kinect_raw, 1);
          double linear_depth = 1.0 / (kinect_raw * -0.0030711016 + 3.3309495161);
          const float k1 = 1.1863;
          const float k2 = 2842.5;
          const float k3 = 0.1236;
          double tan_depth = k3 * tanf(kinect_raw/k2 + k1);
          ntk_dbg_print(linear_depth, 1);
          ntk_dbg_print(tan_depth, 1);
          depth_values.push_back(DepthCalibrationPoint(kinect_raw, p.z));
        }
      }

      ++good_i;
    }
  }
}

void calibrate_kinect_rgb(std::vector< std::vector<Point2f> >& stereo_corners)
{
  std::vector< std::vector<Point2f> > good_corners;
  stereo_corners.resize(global::images_list.size());
  for (int i_image = 0; i_image < global::images_list.size(); ++i_image)
  {
    QString filename = global::images_list[i_image];
    QDir cur_image_dir (global::images_dir.absoluteFilePath(filename));

    std::string full_filename = cur_image_dir.absoluteFilePath("raw/color.png").toStdString();
    ntk_dbg_print(full_filename, 1);
    cv::Mat3b image = imread(full_filename);
    ntk_ensure(image.data, "Could not load color image");

    std::vector<Point2f> current_view_corners;
    calibrationCorners(full_filename, "corners",
                       global::opt_pattern_width(), global::opt_pattern_height(),
                       current_view_corners, image, 1);

    if (current_view_corners.size() == global::opt_pattern_height()*global::opt_pattern_width())
    {
      good_corners.push_back(current_view_corners);
      stereo_corners[i_image] = current_view_corners;
      show_corners(image, current_view_corners, 1);
    }
    else
    {
      ntk_dbg(0) << "Warning: corners not detected";
      stereo_corners[i_image].resize(0);
    }
  }

  std::vector< std::vector<Point3f> > pattern_points;
  calibrationPattern(pattern_points,
                     global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                     good_corners.size());

  ntk_assert(pattern_points.size() == good_corners.size(), "Invalid points size");

  int flags = 0;
  if (global::opt_ignore_distortions())
    flags = CV_CALIB_ZERO_TANGENT_DIST;

  std::vector<Mat> rvecs, tvecs;
  double error = calibrateCamera(pattern_points, good_corners, global::image_size,
                                 global::rgb_intrinsics, global::rgb_distortion,
                                 rvecs, tvecs, flags);

  if (global::opt_ignore_distortions())
    global::rgb_distortion = 0.f;

  ntk_dbg_print(error, 1);
  int good_i = 0;
  foreach_idx(stereo_i, stereo_corners)
  {
    if (stereo_corners[stereo_i].size() > 0)
    {
      QString filename = global::images_list[stereo_i];
      QDir cur_image_dir (global::images_dir.absoluteFilePath(filename));
      std::string full_filename = cur_image_dir.absoluteFilePath("raw/color.png").toStdString();
      cv::Mat3b image = imread(full_filename);
      ntk_ensure(image.data, "Could not load color image");

      cv::Mat3b undistorted_image;
      if (global::opt_ignore_distortions())
        image.copyTo(undistorted_image);
      else
        undistort(image, undistorted_image, global::rgb_intrinsics, global::rgb_distortion);

      std::vector<Point2f> current_view_corners;
      calibrationCorners(full_filename, "corners",
                         global::opt_pattern_width(), global::opt_pattern_height(),
                         current_view_corners, undistorted_image, 1);

      if (current_view_corners.size() == (global::opt_pattern_width()*global::opt_pattern_height()))
      {
        stereo_corners[stereo_i] = current_view_corners;
        show_corners(undistorted_image, stereo_corners[stereo_i], 200);
      }
      else
      {
        stereo_corners[stereo_i].resize(0);
      }
      ++good_i;
    }
  }
}


void calibrate_kinect_stereo(const std::vector< std::vector<Point2f> >& undistorted_rgb_corners,
                             const std::vector< std::vector<Point2f> >& undistorted_depth_corners)
{
  ntk_assert(undistorted_depth_corners.size() == undistorted_rgb_corners.size(), "Size should be equal.");
  std::vector< std::vector<Point2f> > undistorted_good_rgb;
  std::vector< std::vector<Point2f> > undistorted_good_depth;

  foreach_idx(i, undistorted_depth_corners)
  {
    if (undistorted_depth_corners[i].size() > 0 && undistorted_rgb_corners[i].size() > 0)
    {
      ntk_assert(undistorted_depth_corners[i].size() == undistorted_rgb_corners[i].size(),
                 "Sizes should be equal.");
      undistorted_good_depth.push_back(undistorted_depth_corners[i]);
      undistorted_good_rgb.push_back(undistorted_rgb_corners[i]);
    }
  }

  std::vector< std::vector<Point3f> > pattern_points;
  calibrationPattern(pattern_points,
                     global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                     undistorted_good_depth.size());

  cv::Mat E(3,3,CV_64F),F(3,3,CV_64F);
  cv::Mat zero_dist (global::depth_distortion.size(), global::depth_distortion.type());
  zero_dist = Scalar(0);

  stereoCalibrate(pattern_points,
                  undistorted_good_rgb, undistorted_good_depth,
                  global::rgb_intrinsics, zero_dist,
                  global::depth_intrinsics, zero_dist,
                  global::image_size,
                  global::R, global::T, E, F,
                  TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, 1e-6),
                  CALIB_FIX_INTRINSIC);

  double error = computeError(F,
                              undistorted_good_rgb, undistorted_good_depth);
  std::cout << "Average pixel reprojection error: " << error << std::endl;
}

struct DepthOffsetCostFunction : public ntk::CostFunction
{
  DepthOffsetCostFunction(const std::vector<DepthCalibrationPoint>& points)
    : CostFunction(2, points.size()), points(points)
  {}

  virtual void evaluate(const std::vector<double>& input, std::vector<double>& output) const
  {
    double b = input[0];
    double doff = input[1];
    foreach_idx(i, points)
    {
      double kd = points[i].kinect_raw;
      double z = 540.0 * 8.0 * b / (doff - kd);
      output[i] = 1000.0 * (z-points[i].estimated);
    }
  }

  const std::vector<DepthCalibrationPoint>& points;
};

// Taken from http://www.ros.org/wiki/kinect_calibration/technical
void estimate_depth_function(const std::vector<DepthCalibrationPoint>& points)
{
  DepthOffsetCostFunction cost_f(points);
  LevenbergMarquartMinimizer minimizer;
  std::vector<double> x (2);
  x[0] = global::depth_baseline;
  x[1] = global::depth_offset;
  minimizer.minimize(cost_f, x);
  minimizer.diagnoseOutcome();
  global::depth_baseline = x[0];
  global::depth_offset = x[1];
  ntk_dbg_print(global::depth_baseline, 1);
  ntk_dbg_print(global::depth_offset, 1);
}

void writeNestkMatrix()
{
  FileStorage output_file (global::opt_output_file(),
                           CV_STORAGE_WRITE);
  writeMatrix(output_file, "rgb_intrinsics", global::rgb_intrinsics);
  writeMatrix(output_file, "rgb_distortion", global::rgb_distortion);
  writeMatrix(output_file, "depth_intrinsics", global::depth_intrinsics);
  writeMatrix(output_file, "depth_distortion", global::depth_distortion);
  writeMatrix(output_file, "R", global::R);
  writeMatrix(output_file, "T", global::T);
  cv::Mat1i size_matrix(1,2);
  size_matrix(0,0) = global::image_size.width;
  size_matrix(0,1) = global::image_size.height;
  writeMatrix(output_file, "rgb_size", size_matrix);
  writeMatrix(output_file, "raw_rgb_size", size_matrix);
  writeMatrix(output_file, "depth_size", size_matrix);
  writeMatrix(output_file, "raw_depth_size", size_matrix);

  cv::Mat1f depth_calib (1,2);
  depth_calib(0,0) = global::depth_baseline;
  depth_calib(0,1) = global::depth_offset;
  writeMatrix(output_file, "depth_base_and_offset", depth_calib);
  output_file.release();
}

void writeROSMatrix()
{
  // Reload nestk calibration file.
  RGBDCalibration calib;
  calib.loadFromFile(global::opt_output_file());

  cv::Mat1f depth_proj = calib.depth_pose->cvProjectionMatrix();
  cv::Mat1f rgb_proj = calib.rgb_pose->cvProjectionMatrix();
  cv::Mat1f identity(3,3); setIdentity(identity);

  FileStorage ros_depth_file ("calibration_depth.yaml",
                              CV_STORAGE_WRITE);
  writeMatrix(ros_depth_file, "camera_matrix", global::depth_intrinsics);
  writeMatrix(ros_depth_file, "distortion_coefficients", global::depth_distortion);
  writeMatrix(ros_depth_file, "rectification_matrix", identity);
  writeMatrix(ros_depth_file, "projection_matrix", depth_proj);
  ros_depth_file.release();

  FileStorage ros_rgb_file ("calibration_rgb.yaml",
                            CV_STORAGE_WRITE);
  writeMatrix(ros_rgb_file, "camera_matrix", global::rgb_intrinsics);
  writeMatrix(ros_rgb_file, "distortion_coefficients", global::rgb_distortion);
  writeMatrix(ros_rgb_file, "rectification_matrix", identity);
  writeMatrix(ros_rgb_file, "projection_matrix", rgb_proj);
  ros_rgb_file.release();
}

int main(int argc, char** argv)
{
  arg_base::set_help_option("--help");
  arg_parse(argc, argv);
  ntk::ntk_debug_level = 1;

  namedWindow("corners");

  global::images_dir = QDir(global::opt_image_directory());
  ntk_ensure(global::images_dir.exists(), (global::images_dir.absolutePath() + " is not a directory.").toAscii());
  global::images_list = global::images_dir.entryList(QStringList("view????"), QDir::Dirs, QDir::Name);


  std::vector< std::vector<Point2f> > rgb_stereo_corners;
  std::vector< std::vector<Point2f> > depth_stereo_corners;
  std::vector< DepthCalibrationPoint > depth_values;

  calibrate_kinect_rgb(rgb_stereo_corners);
  calibrate_kinect_depth(depth_stereo_corners, depth_values);
  calibrate_kinect_stereo(rgb_stereo_corners, depth_stereo_corners);

  estimate_depth_function(depth_values);

  writeNestkMatrix();
  writeROSMatrix();

  return 0;
}
