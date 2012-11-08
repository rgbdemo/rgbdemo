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
#include <ntk/camera/rgbd_grabber_factory.h>
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
ntk::arg<int> num_devices("--numdevices", "Number of connected Kinects (-1 is all)", -1);
ntk::arg<const char*> bbox("--bbox", "Specifies the initial bounding box (.yml)", 0);

ntk::arg<bool> openni("--openni", "Force OpenNI driver", 0);
ntk::arg<bool> freenect("--freenect", "Force freenect driver", 0);
ntk::arg<bool> kin4win("--kin4win", "Force kin4win driver", 0);
ntk::arg<bool> pmd("--pmd", "Force pmd nano driver", 0);

ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
}

int main (int argc, char** argv)
{
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);
    ntk_debug_level = opt::debug_level();
    cv::setBreakOnError(true);

    QApplication app (argc, argv);

#if 0

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

#endif

    // Config dir is supposed to be next to the binaries.
    QDir prev_dir = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());

    MultipleGrabber* multi_grabber = new MultipleGrabber();
    MultiKinectScanner scanner;

    RGBDGrabberFactory grabber_factory;
    RGBDGrabberFactory::Params params;

    if (opt::directory())
        params.directory = opt::directory();

    if (opt::openni())
        params.type = RGBDGrabberFactory::OPENNI;

    if (opt::freenect())
        params.type = RGBDGrabberFactory::FREENECT;

    if (opt::kin4win())
        params.type = RGBDGrabberFactory::KIN4WIN;

    if (opt::pmd())
        params.type = RGBDGrabberFactory::PMD;

    if (opt::calibration_dir())
        params.calibration_dir = opt::calibration_dir();

    if (opt::high_resolution())
        params.high_resolution = true;

    std::vector<RGBDGrabberFactory::GrabberData> grabbers;
    grabbers = grabber_factory.createGrabbers(params);

    for (int i = 0; i < grabbers.size(); ++i)
    {
        RGBDGrabber* grabber = grabbers[i].grabber;

        if (opt::sync())
            grabber->setSynchronous(true);

        multi_grabber->addGrabber(grabber);
        scanner.addGrabber(grabber);
    }

    QDir::setCurrent(prev_dir.absolutePath());

    GuiMultiKinectController* controller = new GuiMultiKinectController(&scanner);
    if (opt::bbox())
    {
        ntk::Rect3f bbox = readBoundingBoxFromYamlFile(opt::bbox());
        controller->setBoundingBox(bbox, true);
    }

    scanner.plugController(controller);
    controller->scanner().calibratorBlock().setCalibrationPattern(0.034, 10, 7);

    if (opt::sync())
        controller->scanner().setPaused(true);

    scanner.start();
    return app.exec();
}
