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
#include <ntk/gui/image_widget.h>
#include <ntk/mesh/rgbd_modeler.h>

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/thread/event.h>

class QMainWindow;
class RawImagesWindow;
class View3DWindow;
class FiltersWindow;
class ModelAcquisitionWindow;
class ModelAcquisitionController;

namespace ntk
{
class MeshGenerator;
class PlaneEstimator;
}

namespace Ui {
class RawImagesWindow;
}

class GuiController : public ntk::AsyncEventListener
{
    Q_OBJECT

public:
    struct ObjectData
    {
        ntk::Mesh mesh;
        ntk::ImageWidget::TextData text;
        std::vector<cv::Point2i> pixels;
        cv::Rect bbox;
    };

public:
    GuiController(ntk::RGBDGrabber& producer, ntk::RGBDProcessor& processor);
    virtual ~GuiController();

public:
    bool paused() const { return m_paused; }
    void setPaused(bool paused) { m_paused = paused; }
    void processOneFrame() { m_process_one_frame = true; m_grabber.newEvent(); }

    void setPlaneEstimator(ntk::PlaneEstimator& e) { m_plane_estimator = &e; }
    ntk::PlaneEstimator* planeEstimator() { return m_plane_estimator; }

    void setModelAcquisitionController(ModelAcquisitionController& controller);
    ModelAcquisitionController* modelAcquisitionController() { return m_model_acquisition_controller; }

    void setFrameRecorder(ntk::RGBDFrameRecorder& frame_recorder);
    ntk::RGBDFrameRecorder* frameRecorder() { return m_frame_recorder; }

    void setMeshGenerator(ntk::MeshGenerator& generator);

    ntk::MeshGenerator* meshGenerator() { return m_mesh_generator; }
    ntk::RGBDGrabber& grabber() { return m_grabber; }
    ntk::RGBDProcessor& rgbdProcessor() { return m_processor; }

    const ntk::RGBDImage& lastImage() const { return m_last_image; }
    void saveCurrentFrame();

    const ntk::Plane& objectSupportPlane() const { return m_object_plane; }
    ntk::Mesh& modelMesh() { return m_model_mesh; }
    ntk::RGBDImage& modelImage() { return m_model_image; }

    ModelAcquisitionWindow* modelAcquisitionWindow() { return m_model_window; }

    void acquireNewModels();
    void notifyNewModel() { m_new_model = true; }
    void newModelCallback();
    void saveModels();
    void resetModels();

public:
    void toggleShowPlane(bool b) { m_show_plane = b; }
    void toggleRemovePlanePixels(bool b) { m_find_objects = b; }
    void toggleView3d(bool );
    void toggleFilters(bool );
    void toggleModeler(bool );
    void setScreenCaptureMode(bool active) { m_screen_capture_mode = active; }
    void setGrabFrames(bool active) { m_grab_frames = active; }
    void quit();

public slots:
    virtual void onRGBDDataUpdated();
    void on_depth_mouse_moved(int x, int y);

protected:
    virtual void handleAsyncEvent() { onRGBDDataUpdated(); }

protected:
    ntk::RGBDGrabber& m_grabber;
    ntk::RGBDProcessor& m_processor;
    RawImagesWindow* m_raw_images_window;
    View3DWindow* m_view3d_window;
    FiltersWindow* m_filters_window;
    ModelAcquisitionWindow* m_model_window;
    ntk::RGBDFrameRecorder* m_frame_recorder;
    ntk::MeshGenerator* m_mesh_generator;
    ntk::PlaneEstimator* m_plane_estimator;
    ModelAcquisitionController* m_model_acquisition_controller;
    ntk::RGBDImage m_last_image;
    ntk::RGBDImage m_model_image;
    ntk::Mesh m_model_mesh;
    ntk::Plane m_object_plane;
    cv::Rect m_table_roi;
    double m_last_tick;
    int m_frame_counter;
    float m_frame_rate;
    ntk::ScreenGrabber m_modeling_window_grabber;
    ntk::ScreenGrabber m_view3d_window_grabber;
    ntk::ScreenGrabber m_raw_window_grabber;
    bool m_screen_capture_mode;
    bool m_grab_frames;
    bool m_paused;
    bool m_process_one_frame;
    bool m_show_plane;
    bool m_find_objects;
    bool m_new_model;
    std::vector<ObjectData> m_objects;
};


#endif // GUICONTROLLER_H
