/**
 * This file is part of the rgbdemo project.
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
 * Author: Nicolas Burrus <nicolas@burrus.name>, (C) 2010, 2011
 */

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>

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
#include <ntk/mesh/table_object_rgbd_modeler.h>

#include "GuiController.h"
#include "ModelAcquisitionController.h"

#include <QApplication>
#include <QMetaType>

#ifdef NESTK_PRIVATE
# include <ntk/private/mesh/table_object_rgbd_modeler.h>
# include <ntk/private/mesh/silhouette_rgbd_modeler.h>
# include <ntk/private/detection/plane_estimator.h>
#endif

using namespace ntk;
using namespace cv;

namespace opt
{
ntk::arg<int> debug_level("--debug", "Debug level", 1);
ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab1");
ntk::arg<int> first_index("--istart", "First image index", 0);
ntk::arg<const char*> calibration_file("--calibration", "Calibration file (yml)", 0);
ntk::arg<const char*> image("--image", "Fake mode, use given still image", 0);
ntk::arg<const char*> directory("--directory", "Fake mode, use all view???? images in dir.", 0);
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

    RGBDGrabber* grabber = 0;

#ifdef NESTK_USE_OPENNI
    OpenniDriver* ni_driver = 0;
#endif

    bool use_openni = false;
    if (opt::use_kinect())
    {
#ifdef NESTK_USE_OPENNI
        if (opt::use_freenect())
        {
            rgbd_processor = new FreenectRGBDProcessor();
        }
        else
        {
            rgbd_processor = new OpenniRGBDProcessor();
            use_openni = true;
        }
#else
        rgbd_processor = new FreenectRGBDProcessor();
#endif
    }
    else
    {
        rgbd_processor = new RGBDProcessor();
    }

    rgbd_processor->setMaxNormalAngle(40);
    rgbd_processor->setFilterFlag(RGBDProcessor::ComputeMapping, true);
    rgbd_processor->setFilterFlag(RGBDProcessor::FilterThresholdDepth, true);
    rgbd_processor->setMinDepth(0.3);
    rgbd_processor->setMaxDepth(1.5);

    if (opt::image() || opt::directory())
    {
        std::string path = opt::image() ? opt::image() : opt::directory();
        FileGrabber* file_grabber = new FileGrabber(path, is_directory);
        grabber = file_grabber;
        // Image are saved with flipping applied.
        rgbd_processor->setFilterFlag(RGBDProcessor::FlipColorImage, false);
    }
    else if (opt::use_kinect())
    {
        if (opt::use_freenect())
        {
#ifdef NESTK_USE_FREENECT
            FreenectGrabber* k_grabber = new FreenectGrabber();
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
            k_grabber->setTrackUsers(false);
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
        ntk_assert(0, "PMDSDK support not enabled.");
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

    if (opt::sync())
        gui_controller.setPaused(true);

    TableObjectRGBDModeler modeler;
    modeler.setDepthFilling(true);
    modeler.setRemoveSmallStructures(true);

    ModelAcquisitionController* acq_controller = 0;
    acq_controller = new ModelAcquisitionController (gui_controller, modeler);
    acq_controller->setPaused(true);

    RelativePoseEstimator* pose_estimator = new DummyRelativePoseEstimator();

    acq_controller->setPoseEstimator(pose_estimator);
    gui_controller.setModelAcquisitionController(*acq_controller);

    if (mesh_generator)
        gui_controller.setMeshGenerator(*mesh_generator);

    grabber->start();

    app.exec();
    delete mesh_generator;
    delete acq_controller;
}
