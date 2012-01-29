// <nicolas.burrus@uc3m.es>
// <jgbueno@ing.uc3m.es>

#define NESTK_USE_PCL 1

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>
#if defined(USE_PMDSDK) && defined(NESTK_PRIVATE)
# include "pmdsdk2.h"
# include <ntk/private/camera/pmd_grabber.h>
# include <ntk/private/camera/pmd_rgb_grabber.h>
#endif

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <sstream>
#include <iomanip>

#include <ntk/image/sift_gpu.h>
#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#ifdef NESTK_USE_FREENECT
# include <ntk/camera/freenect_grabber.h>
#endif
#ifdef NESTK_USE_OPENNI
# include <ntk/camera/openni_grabber.h>
#endif
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/surfels_rgbd_modeler.h>

#include "GuiController.h"

#include <QApplication>
#include <QMetaType>

#include <ntk/detection/plane_estimator.h>

#ifdef NESTK_PRIVATE
# include <ntk/private/mesh/silhouette_rgbd_modeler.h>
#endif

#ifdef NESTK_USE_PCL
# include <ntk/mesh/table_object_rgbd_modeler.h>
#endif

using namespace ntk;

namespace opt
{
ntk::arg<int> debug_level("--debug", "Debug level", 1);
ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab1");
ntk::arg<int> first_index("--istart", "First image index", 0);
ntk::arg<const char*> calibration_file("--calibration", "Calibration file (yml)", 0);
ntk::arg<const char*> image("--image", "Fake mode, use given still image", 0);
ntk::arg<const char*> directory("--directory", "Fake mode, use all view???? images in dir.", 0);
ntk::arg<const char*> database("--database", "Model database path.", ".");
ntk::arg<int> camera_id("--camera-id", "Camera id for opencv", 0);
ntk::arg<bool> pa10("--pa10", "WARNING: hold the emergency button -- Enable PA10 Controller", 0);
ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
ntk::arg<const char*> pa10_controller("--pa10-controller", "Pa10 trajectory controller", "model");
ntk::arg<bool> use_kinect("--kinect", "Input are kinect images", 1);
ntk::arg<bool> use_freenect("--freenect", "Use libfreenect library", 0);
ntk::arg<bool> use_highres("--highres", "High resolution mode (Nite only)", 0);
ntk::arg<bool> use_icp("--icp", "use ICP refinement", 0);
ntk::arg<const char*> pose_estimator("--pose-estimator",
                                     "Relative pose estimator (file|delta|image)",
                                     "image");
ntk::arg<const char*> model_initial_pose("--pose-initial", "Model initial pose", 0);
ntk::arg<const char*> model_delta_pose("--pose-delta", "Model delta pose", 0);

}

int main (int argc, char** argv)
{
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);
    ntk_debug_level = opt::debug_level();
    cv::setBreakOnError(true);

    QApplication::setGraphicsSystem("raster");
    QApplication app (argc, argv);

    // Opening cameras. Its important to remember that CvCapture has to be done
    // before PMD otherwise will crash
    // 1.- Logitech (openCV)
    // 2.- PMD (pmdlibrary)

    const char* fake_dir = opt::image();
    bool is_directory = opt::directory() != 0;
    if (opt::directory())
        fake_dir = opt::directory();

    ntk::RGBDProcessor* rgbd_processor = 0;

#ifdef NESTK_USE_OPENNI
    OpenniDriver* ni_driver = 0;
#endif

    RGBDGrabber* grabber = 0;

    bool use_openni = false;
    if (opt::use_kinect())
    {
#ifdef NESTK_USE_OPENNI
        if (opt::use_freenect())
        {
            rgbd_processor = new KinectProcessor();
        }
        else
        {
            rgbd_processor = new NiteProcessor();
            use_openni = true;
        }
#else
        rgbd_processor = new KinectProcessor();
#endif
    }
    else
    {
#ifdef USE_PMDSDK
        rgbd_processor = new PmdRgbProcessor();
#else
        rgbd_processor = new RGBDProcessor();
#endif
    }

    rgbd_processor->setMaxNormalAngle(40);
    rgbd_processor->setFilterFlag(RGBDProcessorFlags::ComputeMapping, true);
    rgbd_processor->setFilterFlag(RGBDProcessorFlags::ComputeNormals, true);
    rgbd_processor->setFilterFlag(RGBDProcessorFlags::FilterThresholdDepth, true);
    rgbd_processor->setMinDepth(0.3);
    rgbd_processor->setMaxDepth(1.5);

    if (opt::image() || opt::directory())
    {
        std::string path = opt::image() ? opt::image() : opt::directory();
        FileGrabber* file_grabber = new FileGrabber(path, is_directory);
        grabber = file_grabber;
        // Image are saved with flipping applied.
        rgbd_processor->setFilterFlag(RGBDProcessorFlags::FlipColorImage, false);
    }
    else if (opt::use_kinect())
    {
        if (opt::use_freenect())
        {
#ifdef NESTK_USE_FREENECT
            KinectGrabber* k_grabber = new KinectGrabber();
            k_grabber->initialize();
            k_grabber->setIRMode(false);
            grabber = k_grabber;
#else
            fatal_error("Freenect support not built.");
#endif
        }
        else
        {
#ifdef NESTK_USE_OPENNI
            if (!ni_driver) ni_driver = new OpenniDriver();
            OpenniGrabber* k_grabber = new OpenniGrabber(*ni_driver);
            if (opt::use_highres())
                k_grabber->setHighRgbResolution(true);
            k_grabber->initialize();
            grabber = k_grabber;
#else
            fatal_error("OpenNI support not built, try --freenect.");
#endif
        }
    }
    else
    {
#ifdef USE_PMDSDK
        ntk_dbg(0) << "Creating link with Logitech camera...";
        OpencvGrabber& rgb_grabber = *new OpencvGrabber(cv::Size(800,600));
        rgb_grabber.initialize(opt::camera_id());

        ntk_dbg(0) << "Creating link with PMD camera...";
        PmdGrabber& pmd_grabber = *new PmdGrabber ();
        pmd_grabber.initialize();
        pmd_grabber.setIntegrationTime(600);
        grabber = new ntk::PmdRgbGrabber (rgb_grabber, pmd_grabber);
        pmd_grabber.start();
        rgb_grabber.start();
#else
        ntk_assert(0, "PMDSDK support not enabled.");
#endif
    }

    MeshGenerator* mesh_generator = 0;
    ntk::RGBDCalibration* calib_data = 0;

    if (opt::calibration_file())
    {
        calib_data = new RGBDCalibration();
        calib_data->loadFromFile(opt::calibration_file());
    }
    else if (use_openni)
    {
        calib_data = grabber->calibrationData();
    }

    if (calib_data)
    {
        mesh_generator = new MeshGenerator();
        grabber->setCalibrationData(*calib_data);
    }

    if (opt::sync())
        grabber->setSynchronous(true);

    RGBDFrameRecorder frame_recorder (opt::dir_prefix());
    frame_recorder.setFrameIndex(opt::first_index());
    frame_recorder.setSaveOnlyRaw(true);
    frame_recorder.setUseBinaryRaw(true);

    GuiController gui_controller (*grabber, *rgbd_processor);
    grabber->addEventListener(&gui_controller);
    gui_controller.setFrameRecorder(frame_recorder);

    gui_controller.setDatabasePath(opt::database());

    if (opt::sync())
        gui_controller.setPaused(true);

    if (mesh_generator)
        gui_controller.setMeshGenerator(*mesh_generator);

    grabber->start();

    app.exec();
    delete mesh_generator;
}
