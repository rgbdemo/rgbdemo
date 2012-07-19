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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>, (C) 2012
 */

#include "calibration_common.h"

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>
#include <ntk/camera/pmd_grabber.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <fstream>

#include <QDir>
#include <QDebug>

using namespace ntk;
using namespace cv;

// example command line (for copy-n-paste):
// calibrate_pmdnano --pattern-width 8 --pattern-height 6 images

namespace global
{
ntk::arg<const char*> opt_image_directory(0, "RGBD images directory", 0);
ntk::arg<const char*> opt_output_file("--output", "Calibration file YAML", "calibration.yml");
ntk::arg<const char*> opt_pattern_type("--pattern-type", "Pattern type (chessboard, circles, asymcircles)", "chessboard");
ntk::arg<int> opt_pattern_width("--pattern-width", "Pattern width (number of inner corners)", 10);
ntk::arg<int> opt_pattern_height("--pattern-height", "Pattern height (number of inner corners)", 7);
ntk::arg<float> opt_square_size("--pattern-size", "Square size in used defined scale", 0.025);
ntk::arg<bool> opt_ignore_distortions("--no-undistort", "Ignore distortions (faster processing)", true);
ntk::arg<bool> opt_fix_center("--fix-center", "Do not estimate the central point", true);
std::string output_filename;

PatternType pattern_type;

RGBDCalibration calibration;

double initial_focal_length;

QDir images_dir;
QStringList images_list;
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

    global::images_dir = QDir(global::opt_image_directory());
    ntk_ensure(global::images_dir.exists(), (global::images_dir.absolutePath() + " is not a directory.").toAscii());

    global::images_list = global::images_dir.entryList(QStringList("view????*"), QDir::Dirs, QDir::Name);

    PmdRgbProcessor processor;

    std::vector<ntk::RGBDImage> images;
    loadImageList(global::images_dir, global::images_list,
                  &processor, 0, images);

    if (images.size() < 1)
    {
        ntk_dbg(0) << "No images could be read";
        return 1;
    }

    global::calibration.raw_rgb_size = images[0].rawRgb().size();
    global::calibration.rgb_size = global::calibration.rgb_size;
    global::calibration.raw_depth_size = global::calibration.raw_rgb_size;
    global::calibration.depth_size = global::calibration.raw_depth_size;

    std::vector< std::vector<Point2f> > good_corners;
    std::vector< std::vector<Point2f> > all_corners;
    getCalibratedCheckerboardCorners(images,
                                     global::opt_pattern_width(), global::opt_pattern_height(),
                                     global::pattern_type,
                                     all_corners, good_corners, true);

    calibrate_kinect_rgb(images, good_corners, global::calibration,
                         global::opt_pattern_width(), global::opt_pattern_height(),
                         global::opt_square_size(), global::pattern_type,
                         global::opt_ignore_distortions(),
                         global::opt_fix_center(),
                         0);

    ntk_dbg_print(global::calibration.rgb_intrinsics(0,0), 0);

    global::calibration.rgb_intrinsics.copyTo(global::calibration.depth_intrinsics);
    global::calibration.rgb_distortion.copyTo(global::calibration.depth_distortion);
    global::calibration.updatePoses();

    writeNestkMatrix();
    cv::waitKey(10);
    return 0;
}

