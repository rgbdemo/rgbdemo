#ifndef DETECTION_GUICONTROLLER_H
#define DETECTION_GUICONTROLLER_H

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
class ObjectWindow;

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
    GuiController(ntk::RGBDGrabber& producer, ntk::RGBDProcessor& processor);
    virtual ~GuiController();

public:
    bool paused() const { return m_paused; }
    void setPaused(bool paused) { m_paused = paused; }
    void processOneFrame() { m_process_one_frame = true; m_grabber.newEvent(); }

    void setDatabasePath(const std::string& path);

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

    ObjectWindow* objectWindow() { return m_object_window; }

public:
    void toggleShowPlane(bool b) { m_show_plane = b; }
    void toggleRemovePlanePixels(bool b) { m_find_objects = b; }
    void toggleView3d(bool );
    void toggleObject(bool );
    void toggleFilters(bool );
    void toggleModeler(bool );
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
    View3DWindow* m_view3d_window;
    FiltersWindow* m_filters_window;
    ObjectWindow* m_object_window;
    ntk::RGBDFrameRecorder* m_frame_recorder;
    ntk::MeshGenerator* m_mesh_generator;
    ntk::PlaneEstimator* m_plane_estimator;
    ntk::RGBDImage m_last_image;
    ntk::RGBDImage m_model_image;
    ntk::Mesh m_model_mesh;
    ntk::Plane m_object_plane;
    cv::Rect m_table_roi;
    double m_last_tick;
    int m_frame_counter;
    float m_frame_rate;
    ntk::ScreenGrabber m_view3d_window_grabber;
    ntk::ScreenGrabber m_raw_window_grabber;
    bool m_screen_capture_mode;
    bool m_grab_frames;
    bool m_paused;
    bool m_process_one_frame;
    bool m_show_plane;
    bool m_find_objects;
    bool m_new_model;
};


#endif // DETECTION_GUICONTROLLER_H
