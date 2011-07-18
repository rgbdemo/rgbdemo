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
#include <ntk/camera/rgbd_processor.h>
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

struct DepthCalibrationPoint
{
    DepthCalibrationPoint(double kinect_raw, double estimated)
        : kinect_raw(kinect_raw),
          estimated(estimated)
    {}
    double kinect_raw;
    double estimated;
};

void calibrate_kinect_depth(std::vector< DepthCalibrationPoint >& depth_values)
{
    std::vector< std::vector<Point3f> > pattern_points;
    calibrationPattern(pattern_points,
                       global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                       global::images_list.size());

    for (int i_image = 0; i_image < global::images_list.size(); ++i_image)
    {
        // Generate depth calibration points
        QString filename = global::images_list[i_image];
        QDir cur_image_dir (global::images_dir.absoluteFilePath(filename));
        std::string full_filename = cur_image_dir.absoluteFilePath("raw/color.png").toStdString();
        RGBDImage image;
        OpenniRGBDProcessor processor;
        image.loadFromDir(cur_image_dir.absolutePath().toStdString(), &global::calibration, &processor);

        imshow_normalized("mapped depth", image.mappedDepth());
        imshow("color", image.rgb());

        std::vector<Point2f> current_view_corners;
        calibrationCorners(full_filename, "corners",
                           global::opt_pattern_width(), global::opt_pattern_height(),
                           current_view_corners, image.rgb(), 1, global::pattern_type);

        if (current_view_corners.size() != (global::opt_pattern_width()*global::opt_pattern_height()))
        {
            ntk_dbg(1) << "Corners not detected in " << cur_image_dir.absolutePath().toStdString();
            continue;
        }

        // FIXME: why rvecs and tvecs from calibrateCamera seems to be nonsense ?
        // calling findExtrinsics another time to get good chessboard estimations.

        cv::Mat1f H;
        estimate_checkerboard_pose(pattern_points[0],
                                   current_view_corners,
                                   global::calibration.rgb_intrinsics,
                                   H);
        Pose3D pose;
        pose.setCameraParametersFromOpencv(global::calibration.rgb_intrinsics);
        pose.setCameraTransform(H);
        ntk_dbg_print(pose, 1);

        cv::Mat3b debug_img;
        image.rgb().copyTo(debug_img);
        foreach_idx(pattern_i, pattern_points[0])
        {
            Point3f p = pose.projectToImage(pattern_points[0][pattern_i]);
            float kinect_raw = image.mappedDepth()(p.y, p.x);
            if (kinect_raw < 1e-5) continue;
            float err = kinect_raw-p.z;
            cv::putText(debug_img, format("%d", (int)(err*1000)), Point(p.x, p.y), CV_FONT_NORMAL, 0.4, Scalar(255,0,0));
            ntk_dbg_print(pattern_points[0][pattern_i], 1);
            ntk_dbg_print(p, 1);
            ntk_dbg_print(kinect_raw, 1);
            depth_values.push_back(DepthCalibrationPoint(kinect_raw, p.z));
        }
        imshow("errors", debug_img);
        cv::waitKey(0);
    }
}

void writeNestkMatrix()
{
    global::calibration.saveToFile(global::opt_output_file());
}

void generate_plot(const std::vector<DepthCalibrationPoint>& depth_values)
{
    std::ofstream fkinect("kinect.plot");
    std::ofstream fchess("chessboard.plot");
    std::ofstream ferror("depth_error.plot");
    foreach_idx(i, depth_values)
    {
        fkinect << depth_values[i].kinect_raw << std::endl;
        fchess << depth_values[i].estimated << std::endl;
        ferror << ntk::math::rnd(depth_values[i].kinect_raw*100)/100.0 << " "
               << (depth_values[i].kinect_raw - depth_values[i].estimated) << std::endl;
    }
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

    std::vector<DepthCalibrationPoint> depth_values;
    calibrate_kinect_depth(depth_values);
    generate_plot(depth_values);

    float error_mean = 0;
    foreach_idx(i, depth_values)
    {
        error_mean += (depth_values[i].kinect_raw - depth_values[i].estimated);
    }
    error_mean /= depth_values.size();
    ntk_dbg_print(error_mean, 0);

    writeNestkMatrix();
    return 0;
}
