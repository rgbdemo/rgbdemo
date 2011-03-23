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

#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>

#ifdef NESTK_USE_OPENNI
# include <ntk/camera/nite_rgbd_grabber.h>
#endif

#ifdef NESTK_USE_FREENECT
# include <ntk/camera/kinect_grabber.h>
#endif

#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/surfels_rgbd_modeler.h>
#include "GuiController.h"
#include "ObjectDetector.h"

#include <QApplication>
#include <QMetaType>

using namespace ntk;
using namespace cv;

namespace opt
{
  ntk::arg<const char*> dir_prefix("--prefix", "Directory prefix for output", "grab1");
  ntk::arg<int> first_index("--istart", "First image index", 0);
  ntk::arg<const char*> calibration_file("--calibration", "Calibration file (yml)", 0);
  ntk::arg<const char*> image("--image", "Fake mode, use given still image", 0);
  ntk::arg<const char*> directory("--directory", "Fake mode, use all view???? images in dir.", 0);
  ntk::arg<int> camera_id("--camera-id", "Camera id for opencv", 0);
  ntk::arg<bool> freenect("--freenect", "Force freenect driver", 0);
  ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
  ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
}

int main (int argc, char** argv)
{
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);
  ntk_debug_level = 1;
  cv::setBreakOnError(true);

  QApplication::setGraphicsSystem("raster");
  QApplication app (argc, argv);

  const char* fake_dir = opt::image();
  bool is_directory = opt::directory() != 0;
  if (opt::directory())
    fake_dir = opt::directory();

  ntk::RGBDProcessor* processor = 0;

  RGBDGrabber* grabber = 0;  

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
    NiteRGBDGrabber* k_grabber = new NiteRGBDGrabber();
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
    KinectGrabber* k_grabber = new KinectGrabber();
    k_grabber->initialize();
    k_grabber->setIRMode(false);
    grabber = k_grabber;
  }
#endif

  ntk_ensure(grabber, "Could not create any grabber. Kinect support built?");

  if (use_openni)
  {
    processor = new ntk::NiteProcessor();
  }
  else
  {
    processor = new ntk::KinectProcessor();
  }
  
  if (opt::sync())
    grabber->setSynchronous(true);

  RGBDFrameRecorder frame_recorder (opt::dir_prefix());
  frame_recorder.setFrameIndex(opt::first_index());
  frame_recorder.setSaveOnlyRaw(true);
  frame_recorder.setUseBinaryRaw(true);

  ObjectDetector detector;

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
  else if (QDir::current().exists("kinect_calibration.yml"))
  {
    {
      ntk_dbg(0) << "[WARNING] Using kinect_calibration.yml in current directory";
      ntk_dbg(0) << "[WARNING] use --calibration to specify a different file.";
    }
    calib_data = new RGBDCalibration();
    calib_data->loadFromFile("kinect_calibration.yml");
  }

  if (calib_data)
  {
    mesh_generator = new MeshGenerator();
    grabber->setCalibrationData(*calib_data);
  }

  GuiController gui_controller (*grabber, *processor);
  grabber->addEventListener(&gui_controller);
  gui_controller.setFrameRecorder(frame_recorder);
  gui_controller.setObjectDetector(detector);

  if (opt::sync())
    gui_controller.setPaused(true);

  if (mesh_generator)
  {
    mesh_generator->setUseColor(true);
    gui_controller.setMeshGenerator(*mesh_generator);
  }

  grabber->start();

  app.exec();
  delete mesh_generator;
}
