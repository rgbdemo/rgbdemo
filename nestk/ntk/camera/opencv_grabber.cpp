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

#include "opencv_grabber.h"

#include <ntk/ntk.h>

#include <QDir>

using namespace cv;
using namespace ntk;

OpencvGrabber :: OpencvGrabber(const cv::Size& size)
  : m_image_size(size)
{  
}

void OpencvGrabber :: initialize(int camera_id)
{
  if (!m_logitech_capture.open(camera_id))
    ntk_throw_exception("Could not open logitech camera.");

  m_logitech_capture.set(CV_CAP_PROP_FRAME_WIDTH, m_image_size.width);
  m_logitech_capture.set(CV_CAP_PROP_FRAME_HEIGHT, m_image_size.height);
}

void OpencvGrabber :: run()
{
  cv::Mat3b logitech_image (m_image_size);
  m_rgbd_image.rawRgbRef().create(m_image_size);

  while (!m_should_exit)
  {
    waitForNewEvent();
    m_logitech_capture >> logitech_image;    

    //ntk_assert(logitech_image.cols == m_image_size.width, "Wrong image size");
    if (logitech_image.data == 0)
      ntk_throw_exception("Error adquiring frame from Logitech. Connected?");

    {
      QWriteLocker locker(&m_lock);
      cv::swap(logitech_image, m_rgbd_image.rawRgbRef());
    }

    advertiseNewFrame();
  }
}
