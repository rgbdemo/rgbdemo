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
#include <ntk/utils/opencv_utils.h>

#include <QApplication>

using namespace ntk;
using namespace cv;

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  KinectGrabber grabber;
  grabber.initialize();
  // grabber.start();

  // Set camera tilt.
  grabber.setTiltAngle(15);

  namedWindow("color");

#if 0
  // New opencv window
  namedWindow("color");
  namedWindow("ir");
  namedWindow("depth");
  namedWindow("depth_as_color");

  // Tell the processor to transform raw depth into meters using linear coefficients.
  RGBDProcessor processor;
  processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);

  RGBDImage current_frame;
  cv::Mat3b depth_as_color;
  while (true)
  {
    grabber.waitForNextFrame();
    grabber.copyImageTo(current_frame);
    processor.processImage(current_frame);

    int fps = grabber.frameRate();
    cv::putText(current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", current_frame.rgb());
    imshow("ir", current_frame.intensity());

    // Show depth as normalized gray scale
    imshow_normalized("depth", current_frame.depth());

    // Compute color encoded depth.
    compute_color_encoded_depth(current_frame.depth(), depth_as_color);
    imshow("depth_as_color", depth_as_color);

    unsigned char c = cv::waitKey(10);
    printf("c:%c\n", c);
    if (c == 'f')
      grabber.setIRMode(!grabber.irModeEnabled());
  }
#endif
}

