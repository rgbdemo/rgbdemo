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

#ifndef GUICONTROLLER_H
#define GUICONTROLLER_H

#include <ntk/core.h>
#include "GuiController.h"

#include <ntk/gui/screen_grabber.h>
#include <ntk/mesh/rgbd_modeler.h>

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/thread/event.h>
#include <ntk/pa10/pa10client.h>

class QMainWindow;
class RawImagesWindow;
class View3DWindow;
class FiltersWindow;
class DetectorWindow;
class Pa10ControllerWindow;
class ModelAcquisitionWindow;
class ModelAcquisitionController;
class ObjectDetector;

#ifdef NESTK_PRIVATE
class PedestriansDetectorWindow;
#endif

namespace ntk
{
  class MeshGenerator;
  class PlaneEstimator;
}

namespace Ui {
  class RawImagesWindow;
  class DetectorWindow;
}

class GuiController : public ntk::AsyncEventListener
{
  Q_OBJECT

public:
  GuiController(ntk::RGBDGrabber& producer, ntk::RGBDProcessor& processor);
  virtual ~GuiController();

public:
  void setPaused(bool paused) { m_paused = paused; }
  void processOneFrame() { m_process_one_frame = true; m_grabber.newEvent(); }

  void setPlaneEstimator(ntk::PlaneEstimator& e) { m_plane_estimator = &e; }
  ntk::PlaneEstimator* planeEstimator() { return m_plane_estimator; }

  void setModelAcquisitionController(ModelAcquisitionController& controller);
  ModelAcquisitionController* modelAcquisitionController() { return m_model_acquisition_controller; }

  void setPa10Client(ntk::Pa10client& client,
                     const ntk::Pa10client::s1s2s3e1e2w1w2_t& initial_angles);
  ntk::Pa10client* pa10Client() { return m_pa10_client; }

  void setPa10Angles(const ntk::Pa10client::s1s2s3e1e2w1w2_t& angles);
  const ntk::Pa10client::s1s2s3e1e2w1w2_t& pa10Angles() { return m_pa10_angles; }

  void setFrameRecorder(ntk::RGBDFrameRecorder& frame_recorder);  
  ntk::RGBDFrameRecorder* frameRecorder() { return m_frame_recorder; }

  void setObjectDetector(ObjectDetector& detector);
  ObjectDetector* objectDetector() { return m_object_detector; }

  void setMeshGenerator(ntk::MeshGenerator& generator);

  ntk::MeshGenerator* meshGenerator() { return m_mesh_generator; }
  ntk::RGBDGrabber& grabber() { return m_grabber; }
  ntk::RGBDProcessor& rgbdProcessor() { return m_processor; }

  const ntk::RGBDImage& lastImage() const { return m_last_image; }

  ModelAcquisitionWindow* modelAcquisitionWindow() { return m_model_window; }

public:
  void toggleShowPlane(bool b) { m_show_plane = b; }
  void toggleRemovePlanePixels(bool b) { m_remove_plane_pixels = b; }
  void toggleView3d(bool );
  void toggleObjectDetector(bool );
  void toggleFilters(bool );
  void togglePa10Controller(bool );
  void toggleModeler(bool );
  void setScreenCaptureMode(bool active) { m_screen_capture_mode = active; }
  void setGrabFrames(bool active) { m_grab_frames = active; }
  void quit();
  void togglePedestriansDetector(bool );

public slots:
  virtual void onRGBDDataUpdated();
  void on_depth_mouse_moved(int x, int y);

protected:
  virtual void handleAsyncEvent() { onRGBDDataUpdated(); }

protected:
  ntk::RGBDGrabber& m_grabber;
  ntk::RGBDProcessor& m_processor;
  RawImagesWindow* m_raw_images_window;
  DetectorWindow* m_detector_window;
#ifdef NESTK_PRIVATE
  PedestriansDetectorWindow* m_pedestrians_detector_window;
#endif
  View3DWindow* m_view3d_window;
  FiltersWindow* m_filters_window;
  Pa10ControllerWindow* m_pa10_window;
  ModelAcquisitionWindow* m_model_window;
  ntk::RGBDFrameRecorder* m_frame_recorder;
  ObjectDetector* m_object_detector;
  ntk::MeshGenerator* m_mesh_generator;
  ntk::PlaneEstimator* m_plane_estimator;
  ntk::Pa10client* m_pa10_client;
  ntk::Pa10client::s1s2s3e1e2w1w2_t m_pa10_angles;
  ModelAcquisitionController* m_model_acquisition_controller;
  ntk::RGBDImage m_last_image;
  double m_last_tick;
  int m_frame_counter;
  float m_frame_rate;
  ntk::ScreenGrabber m_modeling_window_grabber;
  ntk::ScreenGrabber m_view3d_window_grabber;
  ntk::ScreenGrabber m_detector_window_grabber;
  ntk::ScreenGrabber m_pedestrians_detector_window_grabber;
  ntk::ScreenGrabber m_raw_window_grabber;
  bool m_screen_capture_mode;
  bool m_grab_frames;
  bool m_paused;
  bool m_process_one_frame;
  bool m_show_plane;
  bool m_remove_plane_pixels;
};

#endif // GUICONTROLLER_H
