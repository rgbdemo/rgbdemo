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

#include <ntk/camera/kinect_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/camera/calibration.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/arg.h>
#include <ntk/geometry/pose_3d.h>

using namespace ntk;
using namespace cv;

namespace opt
{

// Command line argument.
  ntk::arg<const char*> calibration_file("--calibration", "Calibration file", "kinect_calibration.yml");

}

int main(int argc, char** argv)
{
  // Parse command line arguments.
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);

  ntk::ntk_debug_level = 1; // set the debug level

  RGBDCalibration calibration;
  calibration.loadFromFile(opt::calibration_file());

  KinectGrabber grabber;
  grabber.initialize();
  // Tell the grabber that we have calibration data.
  grabber.setCalibrationData(calibration);
  grabber.start();

  // New opencv window
  namedWindow("color");
  namedWindow("depth");
  namedWindow("mapped_color");

  // Tell the processor to transform raw depth into meters using linear coefficients.
  RGBDProcessor processor;
  processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);
  processor.setFilterFlag(RGBDProcessor::FilterMedian, true);

  RGBDImage current_frame;
  cv::Mat3b mapped_color;
  while (true)
  {
    grabber.waitForNextFrame();
    grabber.copyImageTo(current_frame);
    processor.processImage(current_frame);

    ntk_assert(current_frame.calibration(), "Ensure there is calibration data in the image");
    mapped_color.create(current_frame.depth().size());
    mapped_color = Vec3b(0,0,0);

    // equivalent to for(int r = 0; r < im.rows; ++r) for (int c = 0; c < im.cols; ++c)
    for_all_rc(current_frame.depth())
    {
      float depth = current_frame.depth()(r,c);

      // Check for invalid depth.
      if (depth < 1e-5)
        continue;

      // Point in depth image.
      Point3f p_depth (c,r,depth);

      // Point in 3D metric space
      Point3f p3d;
      p3d = current_frame.calibration()->depth_pose->unprojectFromImage(p_depth);
      // Debug output: show p3d if the global debug level is >= 1
      // ntk_dbg_print(p3d, 1);

      // Point in color image
      Point3f p_rgb;
      p_rgb = current_frame.calibration()->rgb_pose->projectToImage(p3d);
      int r_rgb = p_rgb.y;
      int c_rgb = p_rgb.x;
      // Check if the pixel coordinates are valid and set the value.
      if (is_yx_in_range(current_frame.rgb(), r_rgb, c_rgb))
        mapped_color(r, c) = current_frame.rgb()(r_rgb, c_rgb);
    }

    int fps = grabber.frameRate();
    cv::putText(current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", current_frame.rgb());

    // Show depth as normalized gray scale
    imshow_normalized("depth", current_frame.depth());

    // Show color values mapped to depth frame
    imshow("mapped_color", mapped_color);

    cv::waitKey(10);
  }
}

