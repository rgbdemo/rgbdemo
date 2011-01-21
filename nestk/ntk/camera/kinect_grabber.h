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

#ifndef KINECT_GRABBER_H
#define KINECT_GRABBER_H

#include "rgbd_grabber.h"

#include <ntk/core.h>
#include <ntk/utils/qt_utils.h>
#include <ntk/camera/calibration.h>
#include <ntk/thread/event.h>

extern "C" {
#include <libfreenect.h>
}

#include <QThread>

namespace ntk
{

class KinectGrabber : public RGBDGrabber
{
public:
  KinectGrabber()
    : m_depth_transmitted(0),
    m_rgb_transmitted(0),
    f_ctx(0), f_dev(0),
    m_ir_mode(0),
    m_dual_ir_rgb(0)
  {}

  void initialize();
  void depthCallBack(uint16_t *buf, int width, int height);
  void rgbCallBack(uint8_t *buf, int width, int height);
  void irCallBack(uint8_t *buf, int width, int height);
  virtual void setTiltAngle(int angle);
  void setIRMode(bool ir);
  bool irModeEnabled() const { return m_ir_mode; }
  void setDualRgbIR(bool enable);

protected:
  virtual void run();
  void startKinect();

private:
  QReadWriteLock m_kinect_lock;
  RGBDImage m_current_image;
  bool m_depth_transmitted;
  bool m_rgb_transmitted;
  freenect_context *f_ctx;
  freenect_device *f_dev;
  bool m_ir_mode;
  bool m_dual_ir_rgb;
};

} // ntk

#endif // KINECT_GRABBER_H
