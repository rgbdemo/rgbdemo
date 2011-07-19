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

#ifndef NESTK_USE_OPENNI
# error "OpenNI support is required. Enable NESTK_USE_OPENNI."
#endif

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <sstream>
#include <iomanip>

#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/camera/multiple_grabber.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/surfels_rgbd_modeler.h>

#include "GuiController.h"

#include <QApplication>
#include <QMetaType>

using namespace ntk;
using namespace cv;

namespace opt
{
ntk::arg<int> debug_level("--debug", "Debug level", 1);
ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab");
ntk::arg<const char*> calibration_file1("--calibration1", "Calibration file for first Kinect (yml)", 0);
ntk::arg<const char*> calibration_file2("--calibration2", "Calibration file for second Kinect (yml)", 0);
ntk::arg<const char*> calibration_file3("--calibration3", "Calibration file for third Kinect (yml)", 0);
ntk::arg<const char*> calibration_file4("--calibration4", "Calibration file for fourth Kinect (yml)", 0);

ntk::arg<const char*> image1("--image1", "Fake mode, use given still image for first Kinect", 0);
ntk::arg<const char*> image2("--image2", "Fake mode, use given still image for second Kinect", 0);
ntk::arg<const char*> image3("--image3", "Fake mode, use given still image for third Kinect", 0);
ntk::arg<const char*> image4("--image4", "Fake mode, use given still image for fourth Kinect", 0);

ntk::arg<const char*> directory1("--directory1", "Fake mode, use given still directory for first Kinect", 0);
ntk::arg<const char*> directory2("--directory2", "Fake mode, use given still directory for second Kinect", 0);
ntk::arg<const char*> directory3("--directory3", "Fake mode, use given still directory for third Kinect", 0);
ntk::arg<const char*> directory4("--directory4", "Fake mode, use given still directory for fourth Kinect", 0);

ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
ntk::arg<bool> use_highres("--highres", "High resolution mode (Nite only)", 0);
ntk::arg<int> num_devices("--numdevices", "Number of connected Kinects (-1 is all)", -1);
}

int main (int argc, char** argv)
{
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);
    ntk_debug_level = opt::debug_level();
    cv::setBreakOnError(true);

    std::vector<const char*> calibration_files(4);
    calibration_files[0] = opt::calibration_file1();
    calibration_files[1] = opt::calibration_file2();
    calibration_files[2] = opt::calibration_file3();
    calibration_files[3] = opt::calibration_file4();

    std::vector<const char*> image_files(4);
    image_files[0] = opt::image1();
    image_files[1] = opt::image2();
    image_files[2] = opt::image3();
    image_files[3] = opt::image4();

    std::vector<const char*> image_directories(4);
    image_directories[0] = opt::directory1();
    image_directories[1] = opt::directory2();
    image_directories[2] = opt::directory3();
    image_directories[3] = opt::directory4();

    QApplication app (argc, argv);

    ntk::RGBDProcessor* rgbd_processor = new OpenniRGBDProcessor();
    rgbd_processor->setFilterFlag(RGBDProcessor::ComputeMapping, true);

    OpenniDriver ni_driver;
    ntk_ensure(ni_driver.numDevices() >= 1, "No devices connected!");

    if (opt::num_devices() < 0)
        opt::num_devices.value_ = ni_driver.numDevices();

    ntk_ensure(opt::num_devices() <= ni_driver.numDevices(),
               format("Only %d devices detected!", ni_driver.numDevices()).c_str());

    MultipleGrabber* grabber = new MultipleGrabber();
    for (int i = 0; i < opt::num_devices(); ++i)
    {
        RGBDGrabber* dev_grabber = 0;
        if (image_files[i])
        {
            dev_grabber = new FileGrabber(image_files[i], false);
        }
        else if (image_directories[i])
        {
            dev_grabber = new FileGrabber(image_directories[i], true);
        }
        else
        {
            OpenniGrabber* ni_grabber = new OpenniGrabber(ni_driver, i);
            if (opt::use_highres())
            {
                ni_grabber->setHighRgbResolution(true);
            }
            dev_grabber = ni_grabber;
        }
        if (calibration_files[i])
        {
            RGBDCalibration* calib_data = new RGBDCalibration();
            calib_data->loadFromFile(calibration_files[i]);
            dev_grabber->setCalibrationData(*calib_data);
        }
        grabber->addGrabber(dev_grabber);
    }

    MeshGenerator* mesh_generator = new MeshGenerator();
    mesh_generator->setUseColor(true);

    if (opt::sync())
    {
        grabber->setSynchronous(true);
    }

    grabber->connectToDevice();

    RGBDFrameRecorder frame_recorder (opt::dir_prefix());
    frame_recorder.setSaveOnlyRaw(true);
    frame_recorder.setUseBinaryRaw(true);

    GuiController gui_controller (*grabber, *rgbd_processor);
    gui_controller.setFrameRecorder(frame_recorder);
    gui_controller.setMeshGenerator(*mesh_generator);
    // Activate last device by default.
    gui_controller.setActiveDevice(opt::num_devices()-1);

    grabber->addEventListener(&gui_controller);

    if (opt::sync())
        gui_controller.setPaused(true);

    grabber->start();

    app.exec();
}
