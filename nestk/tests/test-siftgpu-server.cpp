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

#include <ntk/image/sift_gpu.h>
#include <ntk/utils/debug.h>

using namespace cv;
using namespace ntk;

int main(int argc, char** argv)
{
  ntk::ntk_debug_level = 1;

  GPUSiftServer server;
  server.run();

  GPUSiftClient client;

  for (int i = 1; i < argc; ++i)
  {
    cv::Mat1b img = imread(argv[i], 0);
    ntk_ensure(img.data, "Could not read image");
    std::vector<KeyPoint> keypoints;
    std::vector<float> descriptors;
    client(img, Mat(), keypoints, descriptors);
    cv::Mat3b debug_img;
    drawKeypoints(img, keypoints, debug_img);
    imshow("sift", debug_img);
    waitKey(0);
  }

  server.stop();
}
