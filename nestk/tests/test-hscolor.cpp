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

#include <ntk/image/color_model.h>
#include <opencv/highgui.h>
#include <ntk/utils/opencv_utils.h>
#include <iostream>

using namespace ntk;
using namespace cv;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: test-hscolor model_image.png new_image.png" << std::endl;
    exit(1);
  }

  const char* ref_image_name = argv[1];
  const char* new_image_name = argv[2];

  cv::Mat3b ref_image = imread(ref_image_name);
  cv::Mat3b new_image = imread(new_image_name);

  HSColorModel model;
  model.build(ref_image, Mat1b() /* empty mask, use all image */);
  model.show();
  cv::waitKey(0);

  cv::Mat1f likelihood_image;
  model.backProject(new_image, likelihood_image);
  cv::Mat1b output;
  imwrite_normalized("likehood.png", likelihood_image);
  threshold(likelihood_image, output, 0.2, 255, cv::THRESH_BINARY);
  imwrite("output.png", output);
}
