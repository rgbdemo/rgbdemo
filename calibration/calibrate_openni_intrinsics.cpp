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
ntk::arg<const char*> opt_image_directory(0, "RGBD images directory", 0);
ntk::arg<const char*> opt_input_file(0, "Input calibration file YAML", "calibration.yml");
ntk::arg<const char*> opt_output_file("--output", "Output YAML filename", "openni_calibration.yml");
ntk::arg<const char*> opt_pattern_type("--pattern-type", "Pattern type (chessboard, circles, asymcircles)", "chessboard");
ntk::arg<int> opt_pattern_width("--pattern-width", "Pattern width (number of inner squares)", 10);
ntk::arg<int> opt_pattern_height("--pattern-height", "Pattern height (number of inner squares)", 7);
ntk::arg<float> opt_square_size("--pattern-size", "Square size in used defined scale", 0.025);
ntk::arg<bool> opt_ignore_distortions("--no-undistort", "Ignore distortions (faster processing)", false);

PatternType pattern_type;

RGBDCalibration calibration;

QDir images_dir;
QStringList images_list;
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
                           current_view_corners, image, 1,
                           global::pattern_type);

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

    ntk_dbg_print(global::opt_square_size(), 0);
    std::vector< std::vector<Point3f> > pattern_points;
    calibrationPattern(pattern_points,
                       global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                       good_corners.size());

    ntk_assert(pattern_points.size() == good_corners.size(), "Invalid points size");

    int flags = CV_CALIB_USE_INTRINSIC_GUESS;
    if (global::opt_ignore_distortions())
        flags = CV_CALIB_ZERO_TANGENT_DIST;

    std::vector<Mat> rvecs, tvecs;
    double error = calibrateCamera(pattern_points, good_corners, global::calibration.rawRgbSize(),
                                   global::calibration.rgb_intrinsics, global::calibration.rgb_distortion,
                                   rvecs, tvecs, flags);

    if (global::opt_ignore_distortions())
        global::calibration.rgb_distortion = 0.f;
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

    global::calibration.loadFromFile(global::opt_input_file());

    global::images_dir = QDir(global::opt_image_directory());
    ntk_ensure(global::images_dir.exists(), (global::images_dir.absolutePath() + " is not a directory.").toAscii());
    global::images_list = global::images_dir.entryList(QStringList("view????"), QDir::Dirs, QDir::Name);

    std::vector< std::vector<Point2f> > rgb_stereo_corners;
    calibrate_kinect_rgb(rgb_stereo_corners);

    double width_ratio = double(global::calibration.rgbSize().width)/global::calibration.depthSize().width;
    double height_ratio = double(global::calibration.rgbSize().height)/global::calibration.depthSize().height;

    global::calibration.rgb_intrinsics.copyTo(global::calibration.depth_intrinsics);
    global::calibration.rgb_distortion.copyTo(global::calibration.depth_distortion);
    global::calibration.depth_intrinsics(0,0) /= width_ratio;
    global::calibration.depth_intrinsics(1,1) /= width_ratio;
    global::calibration.depth_intrinsics(0,2) /= width_ratio;
    global::calibration.depth_intrinsics(1,2) /= width_ratio;

    writeNestkMatrix();
    return 0;
}
