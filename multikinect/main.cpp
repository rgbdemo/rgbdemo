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

#include "GuiMultiKinectController.h"
#include "MultiKinectScanner.h"

#include <QApplication>
#include <QMetaType>

using namespace ntk;
using namespace cv;

class ConsoleMultiKinectController : public MultiKinectController
{
public:
    virtual void onNewImage(ntk::RGBDImageConstPtr image)
    {
        ntk_dbg(1) << "Controller has received an image!";
        ntk_dbg_print(image->rgbWidth(), 1);
    }
};

namespace opt
{
ntk::arg<int> debug_level("--debug", "Debug level", 1);
ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab");
ntk::arg<const char*> calibration_dir("--calibration", "Directory where the calibration files are", 0);
ntk::arg<const char*> directory("--directory", "Fake mode, specify a directory where the streams are", 0);

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

    bool fake_mode = false;
    if (opt::directory())
        fake_mode = true;

    std::vector<std::string> calibration_files;
    std::vector<std::string> image_directories;

    if (fake_mode)
    {
        QDir root_path (opt::directory());
        QStringList devices = root_path.entryList(QStringList("*"), QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
        foreach (QString name, devices)
        {
            QString camera_path = root_path.absoluteFilePath(name);
            if (QDir(camera_path).entryList(QStringList("view*"), QDir::Dirs, QDir::Name).size() == 0)
            {
                ntk_dbg(0) << "Warning, directory " << camera_path << " has no images, skipping.";
                continue;
            }
            ntk_dbg_print(camera_path, 1);
            image_directories.push_back(camera_path.toStdString());
            if (opt::calibration_dir())
                calibration_files.push_back(cv::format("%s/calibration-%s.yml", opt::calibration_dir(), (const char*)name.toAscii()));
        }
    }

    QApplication app (argc, argv);

    ntk::RGBDProcessor* rgbd_processor = new OpenniRGBDProcessor();
    rgbd_processor->setFilterFlag(RGBDProcessorFlags::ComputeMapping, true);

    OpenniDriver* ni_driver = 0;
    if (!fake_mode)
    {
        ni_driver = new OpenniDriver;
        if (opt::num_devices() < 0)
            opt::num_devices.value_ = ni_driver->numDevices();

        if (opt::calibration_dir())
        {
            calibration_files.resize(ni_driver->numDevices());
            for (size_t i = 0; i < ni_driver->numDevices(); ++i)
            {
                std::string name = ni_driver->deviceInfo(i).serial;
                calibration_files[i] = cv::format("%s/calibration-%s.yml", opt::calibration_dir(), name.c_str());
            }
        }
    }
    else
    {
        opt::num_devices.value_ = image_directories.size();
    }

    ntk_ensure(fake_mode || opt::num_devices() <= ni_driver->numDevices(),
               format("Only %d devices detected!", ni_driver->numDevices()).c_str());

    // Config dir is supposed to be next to the binaries.
    QDir prev_dir = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());

    MultipleGrabber* grabber = new MultipleGrabber();
    MultiKinectScanner scanner;
    for (int i = 0; i < opt::num_devices(); ++i)
    {
        RGBDGrabber* dev_grabber = 0;
        if (fake_mode)
        {
            dev_grabber = new FileGrabber(image_directories[i], true);
        }
        else
        {
            ntk_ensure(ni_driver->numDevices() >= 1, "No devices connected!");

            OpenniGrabber* ni_grabber = new OpenniGrabber(*ni_driver, i);

            if (opt::use_highres())
            {
                ni_grabber->setHighRgbResolution(true);
            }
            ni_grabber->setTrackUsers(false);
            dev_grabber = ni_grabber;
        }
        if (opt::calibration_dir())
        {
            RGBDCalibration* calib_data = new RGBDCalibration();
            calib_data->loadFromFile(calibration_files[i].c_str());
            dev_grabber->setCalibrationData(*calib_data);
        }

        if (opt::sync())
            dev_grabber->setSynchronous(true);

        grabber->addGrabber(dev_grabber);
        scanner.addGrabber(dev_grabber);
    }

    QDir::setCurrent(prev_dir.absolutePath());

    GuiMultiKinectController* controller = new GuiMultiKinectController(&scanner);
    scanner.plugController(controller);
    controller->scanner().calibratorBlock().setCalibrationPattern(0.034f, 10, 7);

    if (opt::sync())
        controller->scanner().setPaused(true);

    scanner.start();
    return app.exec();

#if 0

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
    frame_recorder.setSaveIntensity(false);
    frame_recorder.setUseCompressedFormat(false);

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

#endif
}
