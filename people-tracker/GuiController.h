/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#ifndef GUICONTROLLER_H
#define GUICONTROLLER_H

#include <ntk/core.h>
#include "GuiController.h"

#include <ntk/gui/screen_grabber.h>
#include <ntk/mesh/rgbd_modeler.h>

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/thread/event.h>

class QMainWindow;
class RawImagesWindow;
class View3DWindow;
class FiltersWindow;
class PeopleTrackerWindow;
class PeopleTracker;

namespace Ui {
  class RawImagesWindow;
}

class GuiController : public ntk::AsyncEventListener
{
  Q_OBJECT

public:
  GuiController(ntk::RGBDGrabber& producer,
                ntk::RGBDProcessor& processor,
                PeopleTracker& tracker);
  virtual ~GuiController();

public:
  bool isPaused() const { return m_paused; }
  void setPaused(bool paused) { m_paused = paused; }
  void processOneFrame() { m_process_one_frame = true; m_grabber.newEvent(); }

  void setFrameRecorder(ntk::RGBDFrameRecorder& frame_recorder);
  ntk::RGBDFrameRecorder* frameRecorder() { return m_frame_recorder; }

  ntk::RGBDGrabber& grabber() { return m_grabber; }
  ntk::RGBDProcessor& rgbdProcessor() { return m_processor; }

  const ntk::RGBDImage& lastImage() const { return m_last_image; }

public:
  void toggleFilters(bool );
  void togglePeopleTracker(bool active);
  void setScreenCaptureMode(bool active) { m_screen_capture_mode = active; }
  void setGrabFrames(bool active) { m_grab_frames = active; }
  void quit();

public slots:
  virtual void onRGBDDataUpdated();
  void on_depth_mouse_moved(int x, int y);

protected:
  virtual void handleAsyncEvent(ntk::EventListener::Event) { onRGBDDataUpdated(); }

protected:
  ntk::RGBDGrabber& m_grabber;
  ntk::RGBDProcessor& m_processor;
  RawImagesWindow* m_raw_images_window;
  FiltersWindow* m_filters_window;
  PeopleTrackerWindow* m_tracker_window;
  ntk::RGBDFrameRecorder* m_frame_recorder;
  ntk::RGBDImage m_last_image;
  double m_last_tick;
  int m_frame_counter;
  float m_frame_rate;
  ntk::ScreenGrabber m_raw_window_grabber;
  ntk::ScreenGrabber m_tracker_window_grabber;
  bool m_screen_capture_mode;
  bool m_grab_frames;
  bool m_paused;
  bool m_process_one_frame;
};

#endif // GUICONTROLLER_H
