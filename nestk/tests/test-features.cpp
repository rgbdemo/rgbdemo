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

#include <ntk/image/feature.h>
#include <ntk/utils/arg.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/time.h>

using namespace ntk;
using namespace cv;

namespace opt
{
// Command line argument.
ntk::arg<const char*> calibration_file("--calibration", "Calibration file", "kinect_calibration.yml");
ntk::arg<const char*> image(0, "ViewXXXX image", 0);
}

int main(int argc, char** argv)
{
  // Parse command line arguments.
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);
  ntk::ntk_debug_level = 1;

  RGBDImage image;
  image.loadFromDir(opt::image());

  RGBDProcessor rgbd_processor;
  rgbd_processor.processImage(image);

  FeatureSet features;
  TimeCount tc_extract("Extract keypoints");
  features.extractFromImage(image, "SIFTPP", "SIFT");
  tc_extract.stop();
  ntk_dbg_print(features.locations().size(), 1);

  cv::Mat3b display_image;
  features.draw(image.rgb(), display_image);
  imshow("feature points", display_image);
  imwrite("debug_features.png", display_image);
  waitKey(0);
}
