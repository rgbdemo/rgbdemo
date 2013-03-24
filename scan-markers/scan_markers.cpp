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

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>
#ifdef NESTK_USE_OPENNI
# include <ntk/camera/openni_grabber.h>
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
#include <ntk/geometry/incremental_pose_estimator_from_markers.h>

#ifdef NESTK_USE_FREENECT
# include <ntk/camera/freenect_grabber.h>
#endif

#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/surfels_rgbd_modeler.h>
#include "GuiController.h"
#include "ModelAcquisitionController.h"

#include <QApplication>
#include <QMetaType>

using namespace ntk;
using namespace cv;

namespace opt
{
ntk::arg<int> debug_level("--debug", "Debug level", 1);
ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab1");
ntk::arg<const char*> calibration_file("--calibration", "Calibration file (yml)", 0);
ntk::arg<const char*> image("--image", "Fake mode, use given still image", 0);
ntk::arg<const char*> directory("--directory", "Fake mode, use all view???? images in dir.", 0);
ntk::arg<int> camera_id("--camera-id", "Camera id for opencv", 0);
ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
ntk::arg<bool> freenect("--freenect", "Force freenect driver", 0);
ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
ntk::arg<bool> use_icp("--icp", "Use ICP to refine pose estimation", 0);
ntk::arg<float> marker_size("--marker-size", "Marker size in meters", 0.095);
ntk::arg<float> marker_spacing_x("--marker-spacing-x", "Horizontal space between markers in meters", 0.149);
ntk::arg<float> marker_spacing_y("--marker-spacing-y", "Vertical space between markers in meters", 0.278);
}

int main (int argc, char** argv)
{
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);
    ntk_debug_level = opt::debug_level();
    cv::setBreakOnError(true);

    QApplication::setGraphicsSystem("raster");
    QApplication app (argc, argv);

    const char* fake_dir = opt::image();
    bool is_directory = opt::directory() != 0;
    if (opt::directory())
        fake_dir = opt::directory();

    ntk::RGBDProcessor* processor = 0;
    RGBDGrabber* grabber = 0;

#ifdef NESTK_USE_OPENNI
    OpenniDriver* ni_driver = 0;
#endif

    bool use_openni = !opt::freenect();
#ifndef NESTK_USE_OPENNI
    use_openni = false;
#endif

    if (opt::image() || opt::directory())
    {
        std::string path = opt::image() ? opt::image() : opt::directory();
        FileGrabber* file_grabber = new FileGrabber(path, opt::directory() != 0);
        grabber = file_grabber;
    }
#ifdef NESTK_USE_OPENNI
    else if (use_openni)
    {
        // Config dir is supposed to be next to the binaries.
        QDir prev = QDir::current();
        QDir::setCurrent(QApplication::applicationDirPath());
        if (!ni_driver) ni_driver = new OpenniDriver();
        OpenniGrabber* k_grabber = new OpenniGrabber(*ni_driver);
        k_grabber->setTrackUsers(false);
        if (opt::high_resolution())
            k_grabber->setHighRgbResolution(true);
        k_grabber->initialize();
        QDir::setCurrent(prev.absolutePath());
        grabber = k_grabber;
    }
#endif
#ifdef NESTK_USE_FREENECT
    else
    {
        FreenectGrabber* k_grabber = new FreenectGrabber();
        k_grabber->initialize();
        k_grabber->setIRMode(false);
        grabber = k_grabber;
    }
#endif

    ntk_ensure(grabber, "Could not create any grabber. Kinect support built?");

    if (use_openni)
    {
        processor = new ntk::OpenniRGBDProcessor();
    }
    else
    {
        processor = new ntk::FreenectRGBDProcessor();
    }

    if (opt::sync())
        grabber->setSynchronous(true);

    RGBDFrameRecorder frame_recorder (opt::dir_prefix());
    frame_recorder.setSaveOnlyRaw(false);

    ntk::RGBDCalibrationPtr calib_data;
    if (opt::calibration_file())
    {
        calib_data = toPtr(new RGBDCalibration());
        calib_data->loadFromFile(opt::calibration_file());
    }
    else if (use_openni)
    {
        calib_data = grabber->calibrationData();
    }
    else if (QDir::current().exists("kinect_calibration.yml"))
    {
        {
            ntk_dbg(0) << "[WARNING] Using kinect_calibration.yml in current directory";
            ntk_dbg(0) << "[WARNING] use --calibration to specify a different file.";
        }
        calib_data = toPtr(new RGBDCalibration());
        calib_data->loadFromFile("kinect_calibration.yml");
    }

    ntk_ensure(calib_data, "You must specify a calibration file (--calibration)");
    grabber->setCalibrationData(calib_data);

    GuiController gui_controller (*grabber, *processor);
    grabber->addEventListener(&gui_controller);
    gui_controller.setFrameRecorder(frame_recorder);

    if (opt::sync())
        gui_controller.setPaused(true);

    SurfelsRGBDModeler modeler;
    modeler.setMinViewsPerSurfel(2);
    modeler.setResolution(0.0005);
    processor->setFilterFlag(RGBDProcessorFlags::ComputeMapping, true);

    RGBDModeler debug_modeler;
    ModelAcquisitionController* acq_controller = 0;
    acq_controller = new ModelAcquisitionController (gui_controller, modeler);

    IncrementalPoseEstimatorFromMarkers* pose_estimator = 0;
    pose_estimator = new IncrementalPoseEstimatorFromMarkers();

    MarkerSetup setup;
    setup.marker_size = opt::marker_size();
    setup.addMarker(0, cv::Point3f(0,                         0,                       0));
    setup.addMarker(1, cv::Point3f(opt::marker_spacing_x(),   0,                       0));
    setup.addMarker(2, cv::Point3f(0,                         opt::marker_spacing_y(), 0));
    setup.addMarker(3, cv::Point3f(opt::marker_spacing_x(),   opt::marker_spacing_y(), 0));
    pose_estimator->setMarkerSetup(setup);
    // pose_estimator->setMarkerSize(setup.marker_size);
    ntk::Rect3f bbox;
    bbox.x = 0 - opt::marker_size();
    bbox.y = 0 - opt::marker_size();
    bbox.z = -0.01;
    bbox.width = opt::marker_spacing_x() + 2*opt::marker_size();
    bbox.height = opt::marker_spacing_y() + 2*opt::marker_size();
    bbox.depth = 0.5; // 50 cm
    modeler.setBoundingBox(bbox);
    debug_modeler.setBoundingBox(bbox);

    ntk::Plane floor_plane(cv::Vec3f(0, 0, 1), cv::Point3f(0, 0, 0));
    acq_controller->setFloorPlane(floor_plane);

    acq_controller->setPoseEstimator(pose_estimator);
    gui_controller.setModelAcquisitionController(*acq_controller);

    grabber->start();

    app.exec();
    delete acq_controller;
}
