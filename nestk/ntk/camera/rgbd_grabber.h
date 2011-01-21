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

#ifndef NTK_CAMERA_RGBD_GRABBER_H
#define NTK_CAMERA_RGBD_GRABBER_H

#include <ntk/core.h>
#include <ntk/thread/utils.h>
#include <ntk/camera/calibration.h>
#include <ntk/thread/event.h>

#include <QThread>

namespace ntk
{

  class RGBDGrabber : public QThread, public EventBroadcaster, public SyncEventListener
  {
  public:
    RGBDGrabber()
      : m_calib_data(0),
        m_should_exit(0),
        m_last_frame_tick(0),
        m_framerate(0),
        m_frame_count(0)
    {
      setSynchronous(false);
    }

  public:
    virtual void setShouldExit() { m_should_exit = true; }

    virtual void setIntegrationTime(double value) {}
    virtual double integrationTime() const { return -1; }
    virtual void setTiltAngle(int angle) {}

    virtual double frameRate() const { return m_framerate; }
    void advertiseNewFrame();

    void setCalibrationData(const ntk::RGBDCalibration& data)
    { m_calib_data = &data; m_rgbd_image.setCalibration(&data); }

    // Thread safe copy
    void copyImageTo(RGBDImage& image);

    virtual void setSynchronous(bool sync)
    { SyncEventListener::setEnabled(sync); }

    bool isSynchronous() const
    { return SyncEventListener::enabled(); }

    bool hasData() const
    { QReadLocker locker(&m_lock); return m_rgbd_image.depth().data && m_rgbd_image.rgb().data; }

    void waitForNextFrame(int timeout_msecs = 1000)
    {
      m_condition_lock.lock();
      m_condition.wait(&m_condition_lock, timeout_msecs);
      m_condition_lock.unlock();
    }

  protected:
    mutable RecursiveQReadWriteLock m_lock;
    mutable QMutex m_condition_lock;
    QWaitCondition m_condition;
    const ntk::RGBDCalibration* m_calib_data;
    RGBDImage m_rgbd_image;
    bool m_should_exit;
    uint64 m_last_frame_tick;
    double m_framerate;
    int m_frame_count;
  };

} // ntk

#endif // NTK_CAMERA_RGBD_GRABBER_H
