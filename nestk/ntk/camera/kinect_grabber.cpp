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

#include "kinect_grabber.h"
extern "C" {
#include <libfreenect.h>
}
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/camera/rgbd_image.h>

using namespace cv;

namespace ntk
{

  static void kinect_depth_db(freenect_device *dev, void *v_depth, uint32_t timestamp)
  {
    KinectGrabber* grabber = reinterpret_cast<KinectGrabber*>(freenect_get_user(dev));
    uint16_t *depth = reinterpret_cast<uint16_t*>(v_depth);
    grabber->depthCallBack(depth, FREENECT_FRAME_W, FREENECT_FRAME_H);
  }

  static void kinect_video_db(freenect_device *dev, void *rgb, uint32_t timestamp)
    {
	  KinectGrabber* grabber = reinterpret_cast<KinectGrabber*>(freenect_get_user(dev));
    if (grabber->irModeEnabled()) { // ir mode
		uint8_t *ir_cast = reinterpret_cast<uint8_t*>(rgb);
		grabber->irCallBack(ir_cast, FREENECT_FRAME_W, FREENECT_FRAME_H);
	  } else { // rgb mode
	    uint8_t *rgb_cast = reinterpret_cast<uint8_t*>(rgb);
	    grabber->rgbCallBack(rgb_cast, FREENECT_FRAME_W, FREENECT_FRAME_H);
	  }
    }

  void KinectGrabber :: irCallBack(uint8_t *buf, int width, int height)
  {
    ntk_assert(width == m_current_image.rawIntensity().cols, "Bad width");
    ntk_assert(height == m_current_image.rawIntensity().rows, "Bad height");
    float* intensity_buf = m_current_image.rawIntensityRef().ptr<float>();
    for (int i = 0; i < width*height; ++i)
      *intensity_buf++ = *buf++;
    m_rgb_transmitted = false;
  }

  void KinectGrabber :: depthCallBack(uint16_t *buf, int width, int height)
  {
    ntk_assert(width == m_current_image.rawDepth().cols, "Bad width");
    ntk_assert(height == m_current_image.rawDepth().rows, "Bad height");
    float* depth_buf = m_current_image.rawDepthRef().ptr<float>();
    for (int i = 0; i < width*height; ++i)
      *depth_buf++ = *buf++;
    m_depth_transmitted = false;
  }

  void KinectGrabber :: rgbCallBack(uint8_t *buf, int width, int height)
  {
    ntk_assert(width == m_current_image.rawRgb().cols, "Bad width");
    ntk_assert(height == m_current_image.rawRgb().rows, "Bad height");
    std::copy(buf, buf+3*width*height, (uint8_t*)m_current_image.rawRgbRef().ptr());
    cvtColor(m_current_image.rawRgb(), m_current_image.rawRgbRef(), CV_RGB2BGR);
    m_rgb_transmitted = false;
  }

  void KinectGrabber :: setTiltAngle(int angle)
  {
    freenect_set_tilt_degs(f_dev, angle);
  }

  void KinectGrabber :: setDualRgbIR(bool enable)
  {
    m_dual_ir_rgb = enable;
  }

  void KinectGrabber :: setIRMode(bool ir)
  {
    QWriteLocker locker(&m_lock);
    m_ir_mode = ir;
    freenect_stop_video(f_dev);
    if (!m_ir_mode)
      freenect_set_video_format(f_dev, FREENECT_VIDEO_RGB);
    else
      freenect_set_video_format(f_dev, FREENECT_VIDEO_IR_8BIT);
    freenect_start_video(f_dev);
  }

  void KinectGrabber :: startKinect()
  {
    freenect_start_depth(f_dev);
    setIRMode(m_ir_mode);
  }

  void KinectGrabber :: initialize()  
  {
    if (freenect_init(&f_ctx, NULL) < 0)
      fatal_error("freenect_init() failed\n");

    if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
      fatal_error("freenect_open_device() failed\n");

    freenect_set_user(f_dev, this);

    freenect_set_depth_callback(f_dev, kinect_depth_db);
    freenect_set_video_callback(f_dev, kinect_video_db);

    this->setIRMode(m_ir_mode);
  }

  void KinectGrabber :: run()
  {
    m_should_exit = false;
    m_current_image.setCalibration(m_calib_data);
    m_rgbd_image.setCalibration(m_calib_data);

    m_rgbd_image.rawRgbRef() = Mat3b(FREENECT_FRAME_H, FREENECT_FRAME_W);
    m_rgbd_image.rawDepthRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);
    m_rgbd_image.rawIntensityRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);

    m_current_image.rawRgbRef() = Mat3b(FREENECT_FRAME_H, FREENECT_FRAME_W);
    m_current_image.rawDepthRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);
    m_current_image.rawIntensityRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);

    startKinect();
    int64 last_grab_time = 0;

    while (!m_should_exit)
    {
      waitForNewEvent();
      while (m_depth_transmitted || m_rgb_transmitted)
        freenect_process_events(f_ctx);

      // m_current_image.rawDepth().copyTo(m_current_image.rawAmplitudeRef());
      // m_current_image.rawDepth().copyTo(m_current_image.rawIntensityRef());

      {
        int64 grab_time = ntk::Time::getMillisecondCounter();
        ntk_dbg_print(grab_time - last_grab_time, 2);
        last_grab_time = grab_time;
        QWriteLocker locker(&m_lock);
        // FIXME: ugly hack to handle the possible time
        // gaps between rgb and IR frames in dual mode.
        if (m_dual_ir_rgb)
          m_current_image.copyTo(m_rgbd_image);
        else
          m_current_image.swap(m_rgbd_image);
        m_rgb_transmitted = true;
        m_depth_transmitted = true;
      }

      if (m_dual_ir_rgb)
        setIRMode(!m_ir_mode);
      advertiseNewFrame();
#ifdef _WIN32
      // FIXME: this is to avoid GUI freezes with libfreenect on Windows.
      // See http://groups.google.com/group/openkinect/t/b1d828d108e9e69
      Sleep(1);
#endif
    }
  }

} // ntk
