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

#ifndef NTK_OPENCV_GRABBER_H
#define NTK_OPENCV_GRABBER_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_grabber.h>

namespace ntk
{

class OpencvGrabber : public RGBDGrabber
{
public:
  OpencvGrabber(const cv::Size& image_size);
  void initialize(int camera_id = 0);

public:
  const cv::Size& imageSize() const { return m_image_size; }

protected:
  virtual void run();

private:
  cv::Size m_image_size;
  cv::VideoCapture m_logitech_capture;
};

} // ntk

#endif // NTK_OPENCV_GRABBER_H
