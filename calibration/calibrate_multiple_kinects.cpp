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
ntk::arg<const char*> opt_ref_image_directory(0, "RGBD images directory for reference camera", 0);
ntk::arg<const char*> opt_image_directory(0, "RGBD images directory for camera to be calibrated", 0);
ntk::arg<const char*> opt_ref_calibration(0, "Calibration file for reference camera YAML", "calibration_ref.yml");
ntk::arg<const char*> opt_input_calibration(0, "Input initial calibration file YAML", "calibration.yml");
ntk::arg<const char*> opt_output_file("--output", "Output YAML filename", "calibration_multikinect.yml");
ntk::arg<const char*> opt_pattern_type("--pattern-type", "Pattern type (chessboard, circles, asymcircles)", "chessboard");
ntk::arg<int> opt_pattern_width("--pattern-width", "Pattern width (number of inner squares)", 10);
ntk::arg<int> opt_pattern_height("--pattern-height", "Pattern height (number of inner squares)", 7);
ntk::arg<float> opt_square_size("--pattern-size", "Square size in used defined scale", 0.025);

PatternType pattern_type;

RGBDCalibration ref_calibration;
RGBDCalibration calibration;

QDir ref_images_dir;
QStringList ref_images_list;

QDir images_dir;

OpenniRGBDProcessor rgbd_processor;

cv::Mat1d R, T;
}

void get_calibrated_kinect_corners(const QDir& image_dir,
                                   const QStringList& view_list,
                                   RGBDCalibration& calibration,
                                   RGBDProcessor& processor,
                                   std::vector< std::vector<Point2f> >& output_corners)
{
    std::vector< std::vector<Point2f> > good_corners;
    output_corners.resize(view_list.size());
    for (int i_image = 0; i_image < view_list.size(); ++i_image)
    {
        QString filename = view_list[i_image];
        QDir cur_image_dir (image_dir.absoluteFilePath(filename));

        RGBDImage image;
        image.loadFromDir(cur_image_dir.absolutePath().toStdString(), &calibration, &processor);

        std::vector<Point2f> current_view_corners;
        calibrationCorners(filename.toStdString(), "corners",
                           global::opt_pattern_width(), global::opt_pattern_height(),
                           current_view_corners, image.rgb(), 1,
                           global::pattern_type);

        if (current_view_corners.size() == global::opt_pattern_height()*global::opt_pattern_width())
        {
            output_corners[i_image] = current_view_corners;
            show_corners(image.rgb(), current_view_corners, 1);
        }
        else
        {
            ntk_dbg(0) << "Warning: corners not detected";
            output_corners[i_image].resize(0);
        }
    }
}

void calibrate_kinect_stereo(const std::vector< std::vector<Point2f> >& undistorted_ref_corners,
                             const std::vector< std::vector<Point2f> >& undistorted_corners)
{
  ntk_assert(undistorted_ref_corners.size() == undistorted_corners.size(), "Size should be equal.");
  std::vector< std::vector<Point2f> > undistorted_good_corners;
  std::vector< std::vector<Point2f> > undistorted_good_ref_corners;

  foreach_idx(i, undistorted_ref_corners)
  {
    if (undistorted_ref_corners[i].size() > 0 && undistorted_corners[i].size() > 0)
    {
      ntk_assert(undistorted_ref_corners[i].size() == undistorted_corners[i].size(),
                 "Sizes should be equal.");
      undistorted_good_ref_corners.push_back(undistorted_ref_corners[i]);
      undistorted_good_corners.push_back(undistorted_corners[i]);
    }
  }

  std::vector< std::vector<Point3f> > pattern_points;
  calibrationPattern(pattern_points,
                     global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                     undistorted_good_ref_corners.size());

  cv::Mat E(3,3,CV_64F),F(3,3,CV_64F);
  cv::Mat zero_dist (global::calibration.depth_distortion.size(), global::calibration.depth_distortion.type());
  zero_dist = Scalar(0);

  stereoCalibrate(pattern_points,
                  undistorted_good_ref_corners, undistorted_good_corners,
                  global::calibration.rgb_intrinsics, zero_dist,
                  global::calibration.rgb_intrinsics, zero_dist,
                  global::calibration.rgbSize(),
                  global::R, global::T, E, F,
                  TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, 1e-6),
                  CALIB_FIX_INTRINSIC);

  // OpenCV coords has y down and z toward scene.
  // OpenGL classical 3d coords has y up and z backwards
  // This is the transform matrix.

  cv::Mat1d to_gl_base(3,3); setIdentity(to_gl_base);
  to_gl_base(1,1) = -1;
  to_gl_base(2,2) = -1;

  cv::Mat1d new_R = to_gl_base.inv() * global::R * to_gl_base;
  cv::Mat1d new_T = to_gl_base * (global::T);

  new_R.copyTo(global::R);
  new_T.copyTo(global::T);

  double error = computeError(F,
                              undistorted_good_ref_corners, undistorted_good_corners);
  std::cout << "Average pixel reprojection error: " << error << std::endl;
}


void writeNestkMatrix()
{
    global::calibration.saveToFile(global::opt_output_file());
}

int main(int argc, char** argv)
{
    arg_base::set_help_option("--help");
    arg_parse(argc, argv);
    ntk::ntk_debug_level = 1;   

    namedWindow("corners");

    if      (std::string(global::opt_pattern_type()) == "chessboard") global::pattern_type = PatternChessboard;
    else if (std::string(global::opt_pattern_type()) == "circles") global::pattern_type = PatternCircles;
    else if (std::string(global::opt_pattern_type()) == "asymcircles") global::pattern_type = PatternAsymCircles;
    else fatal_error(format("Invalid pattern type: %s\n", global::opt_pattern_type()).c_str());

    global::ref_calibration.loadFromFile(global::opt_ref_calibration());
    global::calibration.loadFromFile(global::opt_input_calibration());

    global::ref_images_dir = QDir(global::opt_ref_image_directory());
    ntk_ensure(global::ref_images_dir.exists(), (global::ref_images_dir.absolutePath() + " is not a directory.").toAscii());

    global::images_dir = QDir(global::opt_image_directory());
    ntk_ensure(global::images_dir.exists(), (global::images_dir.absolutePath() + " is not a directory.").toAscii());

    global::ref_images_list = global::ref_images_dir.entryList(QStringList("view????"), QDir::Dirs, QDir::Name);

    std::vector< std::vector<Point2f> > ref_corners;
    get_calibrated_kinect_corners(global::ref_images_dir,
                                  global::ref_images_list,
                                  global::ref_calibration,
                                  global::rgbd_processor,
                                  ref_corners);

    std::vector< std::vector<Point2f> > corners;
    get_calibrated_kinect_corners(global::images_dir,
                                  global::ref_images_list,
                                  global::calibration,
                                  global::rgbd_processor,
                                  corners);

    calibrate_kinect_stereo(ref_corners, corners);
    global::R.copyTo(global::calibration.R_extrinsics);
    global::T.copyTo(global::calibration.T_extrinsics);

    writeNestkMatrix();
    return 0;
}
