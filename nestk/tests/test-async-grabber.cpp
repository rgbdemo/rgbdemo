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

#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/rgbd_processor.h>

#include <QApplication>

using namespace ntk;
using namespace cv;

class ImageHandler : public AsyncEventListener
{
public:
  ImageHandler(OpencvGrabber& grabber) : m_grabber(grabber)
  {
    namedWindow("color");
  }

public:
  // From AsyncUpdater. Will be called whener the published
  // has a new image.
  virtual void handleAsyncEvent() { handleNewImage(); }

  void handleNewImage()
  {
    m_grabber.copyImageTo(m_current_frame);
    m_processor.processImage(m_current_frame);
    int fps = m_grabber.frameRate();
    cv::putText(m_current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", m_current_frame.rgb());
    cv::waitKey(10);
  }

private:
  OpencvGrabber& m_grabber;
  RGBDImage m_current_frame;
  RGBDProcessor m_processor;
};

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  OpencvGrabber grabber(cv::Size(640, 480));
  grabber.initialize(0 /* camera id */);

  ImageHandler handler(grabber);

  // Register image handler as a listener of grabbed data.
  grabber.addEventListener(&handler);

  // Start the grabbing thread.
  grabber.start();

  // Launch QT main loop.
  return app.exec();
}

