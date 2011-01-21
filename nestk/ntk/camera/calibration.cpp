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

#include "calibration.h"

#include <ntk/ntk.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/utils.h>
#include <ntk/mesh/mesh.h>

#include <opencv/highgui.h>

using namespace ntk;
using namespace cv;

class PlaneEstimator;
class ModelAcquisitionWindow;
class ModelAcquisitionController;
class ObjectDetector;

namespace ntk
{

RGBDCalibration :: ~RGBDCalibration()
{
  delete depth_pose;
  delete rgb_pose;
}

void RGBDCalibration :: loadFromFile(const char* filename)
{
  QFileInfo f (filename);
  ntk_throw_exception_if(!f.exists(), "Could not find calibration file.");
  cv::FileStorage calibration_file (filename, CV_STORAGE_READ);
  readMatrix(calibration_file, "rgb_intrinsics", rgb_intrinsics);
  readMatrix(calibration_file, "rgb_distortion", rgb_distortion);
  zero_rgb_distortion = rgb_distortion(0,0) < 1e-5 ? true : false;
  readMatrix(calibration_file, "depth_intrinsics", depth_intrinsics);
  readMatrix(calibration_file, "depth_distortion", depth_distortion);
  zero_depth_distortion = depth_distortion(0,0) < 1e-5 ? true : false;;
  readMatrix(calibration_file, "R", R);
  readMatrix(calibration_file, "T", T);
  cv::Mat1i size_mat;
  readMatrix(calibration_file, "rgb_size", size_mat);
  rgb_size = cv::Size(size_mat(0,0), size_mat(0,1));
  readMatrix(calibration_file, "raw_rgb_size", size_mat);
  raw_rgb_size = cv::Size(size_mat(0,0), size_mat(0,1));
  cv::Mat1i rgb_size_mat;
  readMatrix(calibration_file, "depth_size", size_mat);
  depth_size = cv::Size(size_mat(0,0), size_mat(0,1));
  readMatrix(calibration_file, "raw_depth_size", size_mat);
  raw_depth_size = cv::Size(size_mat(0,0), size_mat(0,1));
  calibration_file.release();

  cv::Mat1f depth_calib (1,2);
  try {
    readMatrix(calibration_file, "depth_base_and_offset", depth_calib);
    depth_baseline = depth_calib(0,0);
    depth_offset = depth_calib(0,1);
  }
  catch(...)
  {}

  depth_pose = new Pose3D();
  depth_pose->setCameraParametersFromOpencv(depth_intrinsics);

  rgb_pose = new Pose3D();
  rgb_pose->setRightCameraParametersFromOpencv(rgb_intrinsics, R, T);

  initUndistortRectifyMap(rgb_intrinsics, rgb_distortion,
                          Mat(), rgb_intrinsics, rgb_size, CV_16SC2,
                          rgb_undistort_map1, rgb_undistort_map2);

  initUndistortRectifyMap(depth_intrinsics, depth_distortion,
                          Mat(), depth_intrinsics, depth_size, CV_16SC2,
                          depth_undistort_map1, depth_undistort_map2);
}

} // ntk

namespace ntk
{

void calibrationCorners(const std::string& image_name,
                        const std::string& window_name,
                        int pattern_width, int pattern_height,
                        std::vector<Point2f>& corners,
                        const cv::Mat& image,
                        float scale_factor)
{
  Size pattern_size (pattern_width, pattern_height);

  cv::Mat scaled_image;

  if (ntk::flt_eq(scale_factor, 1))
  {
    scaled_image = image.clone();
  }
  else
  {
    cv::resize(image, scaled_image,
               Size(image.cols*scale_factor, image.rows*scale_factor),
               scale_factor, scale_factor, cv::INTER_CUBIC);
  }

  int flags = CV_CALIB_CB_NORMALIZE_IMAGE|CV_CALIB_CB_ADAPTIVE_THRESH;
  bool ok = findChessboardCorners(scaled_image,
                                  pattern_size,
                                  corners,
                                  flags);
  if (!ok)
  {
    flags = CV_CALIB_CB_NORMALIZE_IMAGE;
    ok = findChessboardCorners(scaled_image,
                               pattern_size,
                               corners,
                               flags);
  }

  if (!ok)
  {
    flags = CV_CALIB_CB_ADAPTIVE_THRESH;
    ok = findChessboardCorners(scaled_image,
                               pattern_size,
                               corners,
                               flags);
  }

  if (!ok)
  {
    flags = 0;
    ok = findChessboardCorners(scaled_image,
                               pattern_size,
                               corners,
                               flags);
  }

  cv::Mat draw_image = scaled_image;

  if (ok)
  {
    cv::Mat gray_image;
    cvtColor(image, gray_image, CV_BGR2GRAY);
    cornerSubPix(gray_image, corners, Size(5,5), Size(-1,-1),
                 cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

    cv::Mat corner_matrix(corners.size(), 1, CV_32FC2);
    for (int row = 0; row < corners.size(); ++row)
      corner_matrix.at<Point2f>(row,0) = corners[row];

    drawChessboardCorners(draw_image, pattern_size, corner_matrix, ok);
    ntk_dbg_print(image_name, 1);
    imwrite(image_name + ".corners.png", draw_image);
    imshow(window_name, draw_image);
    waitKey(10);

    for (int i = 0; i < corners.size(); ++i)
    {
      corners[i].x /= scale_factor;
      corners[i].y /= scale_factor;
    }
  }

  if (!ok)
  {
    corners.clear();
    return;
  }
}

void crop_image(cv::Mat& image, cv::Size s)
{
  cv::Mat roi = image(cv::Rect((image.cols-s.width)/2.0,
                               (image.rows-s.height)/2.0,
                               s.width,
                               s.height));
  cv::Mat new_image = cv::Mat(s, image.type());
  roi.copyTo(new_image);
  image = new_image;
}

void depth_distortionance_to_depth(cv::Mat1f& depth_im,
                                   const RGBDCalibration& calib,
                                   const cv::Mat1f& amplitude)
{
  double cx = calib.depth_intrinsics(0,2);
  double cy = calib.depth_intrinsics(1,2);
  double fx = calib.depth_intrinsics(0,0);
  double fy = calib.depth_intrinsics(1,1);

  for_all_rc(depth_im)
  {
    double orig_depth = depth_im(r,c);

    double dx = c-cx;
    double dy = r-cy;

    Point3f v(dx/fx, dy/fy, 1);
    double norm = sqrt(v.dot(v));
    v = v * (1.0/norm);
    v *= orig_depth;

    double depth_z = v.z;
    depth_im(r,c) = depth_z;
  }
}

void calibrationPattern(std::vector< std::vector<Point3f> >& output,
                        int pattern_width,
                        int pattern_height,
                        float square_size,
                        int nb_images)
{
  const int nb_corners = pattern_width * pattern_height;

  output.resize(nb_images);
  for(int i = 0; i < nb_images; ++i)
  {
    output[i].resize(nb_corners);
    for(int j = 0; j < pattern_height; ++j)
      for(int k = 0; k < pattern_width; ++k)
      {
        output[i][j*pattern_width+k] = Point3f(k*square_size, j*square_size, 0);
      }
  }
}

void estimate_checkerboard_pose(const std::vector<Point3f>& model,
                                const std::vector<Point2f>& img_points,
                                const cv::Mat1d& calib_matrix,
                                cv::Mat1f& H)
{
  cv::Mat1f to_open_cv (4,4);
  setIdentity(to_open_cv);
  to_open_cv(1,1) = -1;
  to_open_cv(2,2) = -1;
  cv::Mat1f from_open_cv = to_open_cv.inv();

  Mat3f model_mat(model.size(), 1); CvMat c_model_mat = model_mat;
  for_all_rc(model_mat) model_mat(r, 0) = Vec3f(model[r].x, -model[r].y, -model[r].z);

  // First image, for model pose.

  Mat2f point_mat(img_points.size(), 1); CvMat c_point_mat = point_mat;
  for_all_rc(point_mat) point_mat(r, 0) = Vec2f(img_points[r].x, img_points[r].y);

  Mat1f rvec (3,1); rvec = 0; CvMat c_rvec = rvec;
  Mat1f tvec (3,1); tvec = 0; CvMat c_tvec = tvec;

  CvMat c_calib_mat = calib_matrix;
  cvFindExtrinsicCameraParams2(&c_model_mat,
                               &c_point_mat,
                               &c_calib_mat,
                               0, &c_rvec, &c_tvec);

  cv::Mat1f rot(3,3); CvMat c_rot = rot;
  cvRodrigues2(&c_rvec, &c_rot);

  H = cv::Mat1f(4,4);
  setIdentity(H);
  cv::Mat1f H_rot = H(Rect(0,0,3,3));
  rot.copyTo(H_rot);
  H(0,3) = tvec(0,0);
  H(1,3) = tvec(1,0);
  H(2,3) = tvec(2,0);
  ntk_dbg_print(H, 1);

  H = from_open_cv * H * to_open_cv;
}

} // ntk
