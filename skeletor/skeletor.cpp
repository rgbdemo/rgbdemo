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
#include <ntk/camera/openni_grabber.h>
#include <ntk/mesh/mesh_generator.h>
#include "GuiController.h"

#include <QApplication>
#include <QMetaType>

using namespace ntk;
using namespace cv;

namespace opt
{
ntk::arg<int> debug_level("--debug", "Debug level", 1);
ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab1");
ntk::arg<const char*> calibration_file("--calibration", "Calibration file (yml)", 0);
ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
ntk::arg<bool> high_resolution("--highres", "Low resolution color image.", 0);
}

int main (int argc, char** argv)
{
    arg_base::set_help_option("-h");
    arg_parse(argc, argv);
    ntk_debug_level = opt::debug_level();
    cv::setBreakOnError(true);

    QApplication::setGraphicsSystem("raster");
    QApplication app (argc, argv);

    ntk::RGBDProcessor* processor = new OpenniRGBDProcessor();

    OpenniDriver ni_driver;

    QDir prev = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());
    OpenniGrabber* grabber = new OpenniGrabber(ni_driver);
    if (opt::high_resolution())
        grabber->setHighRgbResolution(true);
    grabber->initialize();
    QDir::setCurrent(prev.absolutePath());

    if (opt::sync())
        grabber->setSynchronous(true);

    RGBDFrameRecorder frame_recorder (opt::dir_prefix());
    frame_recorder.setSaveOnlyRaw(false);

    ntk::RGBDCalibration* calib_data = grabber->calibrationData();

    GuiController gui_controller (*grabber, *processor);
    grabber->addEventListener(&gui_controller);
    gui_controller.setFrameRecorder(frame_recorder);

    if (opt::sync())
        gui_controller.setPaused(true);

    grabber->start();

    app.exec();
}
