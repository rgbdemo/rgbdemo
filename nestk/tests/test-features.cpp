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

/*!
 * Usage example:
 * test-features --detector FAST --extractor BRIEF64 --match-threshold 0.55 view0000 view0004
 */

namespace opt
{
// Command line argument.
ntk::arg<const char*> calibration_file("--calibration", "Calibration file", 0);
ntk::arg<const char*> detector("--detector", "Detector type: SURF, FAST, SIFT, GPUSIFT, SIFTPP", "SURF");
ntk::arg<const char*> extractor("--extractor", "Extrator type: SURF64, SURF128, SIFT, BRIEF32, BRIEF64", "SURF128");
ntk::arg<float> match_threshold("--match-threshold", "Threshold on dist ratio for matching", 0.8*0.8);
ntk::arg<const char*> image1(0, "ViewXXXX image 1", 0);
ntk::arg<const char*> image2(0, "ViewXXXX image 2", 0);
}

int main(int argc, char** argv)
{
  // Parse command line arguments.
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);
  ntk::ntk_debug_level = 1;

  RGBDCalibration* calib = 0;
  if (opt::calibration_file())
  {
    calib = new RGBDCalibration();
    calib->loadFromFile(opt::calibration_file());
  }

  RGBDImage image1;
  image1.loadFromDir(opt::image1(), calib);

  RGBDImage image2;
  image2.loadFromDir(opt::image2(), calib);

  RGBDProcessor rgbd_processor;
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);

  rgbd_processor.processImage(image1);
  rgbd_processor.processImage(image2);

  FeatureSetParams feature_params(opt::detector(), opt::extractor(), true);

  FeatureSet features1;
  {
    TimeCount tc_extract("Extract keypoints");
    features1.extractFromImage(image1, feature_params);
    tc_extract.stop();
    ntk_dbg_print(features1.locations().size(), 1);

    cv::Mat3b display_image1;
    features1.draw(image1.rgb(), display_image1);
    imshow("feature points 1", display_image1);
    imwrite("debug_features_1.png", display_image1);
  }

  FeatureSet features2;
  {
    TimeCount tc_extract("Extract keypoints");
    features2.extractFromImage(image2, feature_params);
    tc_extract.stop();
    ntk_dbg_print(features2.locations().size(), 1);

    cv::Mat3b display_image2;
    features2.draw(image2.rgb(), display_image2);
    imshow("feature points 1", display_image2);
    imwrite("debug_features_1.png", display_image2);
  }

  std::vector<DMatch> matches;
  features1.matchWith(features2, matches, opt::match_threshold());
  ntk_dbg_print(matches.size(), 1);

  cv::Mat3b display_image;
  features1.drawMatches(image1.rgb(), image2.rgb(), features2, matches, display_image);
  imshow("matches", display_image);

  while ((waitKey(0) & 0xff) != 27)
    ;
}
