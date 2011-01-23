// <nicolas.burrus@uc3m.es>
// <jgbueno@ing.uc3m.es>


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

#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/camera/kinect_grabber.h>
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
  ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
}

int main (int argc, char** argv)
{
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);
  ntk_debug_level = 1;
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

  ntk::RGBDProcessor rgbd_processor;
  rgbd_processor.setMaxNormalAngle(40);
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeMapping, false);

  RGBDGrabber* grabber = 0;
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);
  rgbd_processor.setFilterFlag(RGBDProcessor::NoAmplitudeIntensityUndistort, true);

  if (opt::image() || opt::directory())
  {
    std::string path = opt::image() ? opt::image() : opt::directory();
    FileGrabber* file_grabber = new FileGrabber(path, opt::directory() != 0);
    grabber = file_grabber;
  }
  else
  {
    KinectGrabber* k_grabber = new KinectGrabber();
    k_grabber->initialize();
    k_grabber->setIRMode(false);
    grabber = k_grabber;
  }

  if (opt::sync())
    grabber->setSynchronous(true);

  RGBDFrameRecorder frame_recorder (opt::dir_prefix());
  frame_recorder.setFrameIndex(opt::first_index());
  frame_recorder.setSaveOnlyRaw(false);

  ObjectDetector detector;

  MeshGenerator* mesh_generator = 0;
  ntk::RGBDCalibration calib_data;
  if (opt::calibration_file())
  {
    calib_data.loadFromFile(opt::calibration_file());
    mesh_generator = new MeshGenerator();
    grabber->setCalibrationData(calib_data);
  }

  GuiController gui_controller (*grabber, rgbd_processor);
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
