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

#include <ntk/image/sift_gpu.h>
#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#ifdef USE_FREENECT
# include <ntk/camera/kinect_grabber.h>
#endif
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/surfels_rgbd_modeler.h>
#include <ntk/pa10/pa10client.h>
#include "GuiController.h"
#include "ObjectDetector.h"
#include "ModelAcquisitionController.h"

#include <QApplication>
#include <QMetaType>

#ifdef NESTK_PRIVATE
# include <ntk/private/mesh/silhouette_rgbd_modeler.h>
# include <ntk/private/detection/plane_estimator.h>
#endif

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
  ntk::arg<bool> pa10("--pa10", "WARNING: hold the emergency button -- Enable PA10 Controller", 0);
  ntk::arg<bool> sync("--sync", "Synchronization mode", 0);
  ntk::arg<bool> pose_controller("--pose-controller", "Pose acquisition trajectory", 0);
  ntk::arg<bool> use_kinect("--kinect", "Input are kinect images", 0);
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
  ntk_debug_level = 1;
  cv::setBreakOnError(true);

  GPUSiftServer server;
  if (server.isSupported())
    server.run();

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

  if (opt::use_kinect())
  {
    rgbd_processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);
    rgbd_processor.setFilterFlag(RGBDProcessor::NoAmplitudeIntensityUndistort, true);
  }
  else
  {
    rgbd_processor.setFilterFlag(RGBDProcessor::FixGeometry, true);
  }

  if (opt::image() || opt::directory())
  {
    std::string path = opt::image() ? opt::image() : opt::directory();
    FileGrabber* file_grabber = new FileGrabber(path, opt::directory() != 0);
    grabber = file_grabber;
  }
  else if (opt::use_kinect())
  {
#ifdef USE_FREENECT
    KinectGrabber* k_grabber = new KinectGrabber();
    k_grabber->initialize();
    k_grabber->setIRMode(false);
    grabber = k_grabber;
#else
    fatal_error("Kinect support not build.");
#endif
  }
  else
  {
    rgbd_processor.setFilterFlag(RGBDProcessor::FlipColorImage, true);

#ifdef USE_PMDSDK
    ntk_dbg(0) << "Creating link with Logitech camera...";
    OpencvGrabber& rgb_grabber = *new OpencvGrabber(cv::Size(800,600));
    rgb_grabber.initialize(opt::camera_id());

    ntk_dbg(0) << "Creating link with PMD camera...";
    PmdGrabber& pmd_grabber = *new PmdGrabber ();
    pmd_grabber.initialize();
    // pmd_grabber.setIntegrationTime(1000);
    grabber = new ntk::PmdRgbGrabber (rgb_grabber, pmd_grabber);
    pmd_grabber.start();
    rgb_grabber.start();
#else
    ntk_assert(0, "PMDSDK support not enabled.");
#endif
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

  Pa10client* pa10_client = 0;
  Pa10client::s1s2s3e1e2w1w2_t initial_angles = {110, -90, -90, 90, -90, -90.0, 0.0};
  if (opt::pa10())
  {
    // Connecting to PA10.
    pa10_client = new Pa10client();
    pa10_client->connectionAddress = "163.117.150.21";
    pa10_client->pa10serverConnectionPort = 12345;
    bool ok = pa10_client->start();
    ntk_ensure(ok, "Could not connect to PA10");
  }

  GuiController gui_controller (*grabber, rgbd_processor);
  grabber->addEventListener(&gui_controller);
  gui_controller.setFrameRecorder(frame_recorder);
  gui_controller.setObjectDetector(detector);

#ifdef NESTK_PRIVATE
  ntk::PlaneEstimator plane_estimator;
  gui_controller.setPlaneEstimator(plane_estimator);
#endif

  if (opt::sync())
    gui_controller.setPaused(true);

#if 1
  SurfelsRGBDModeler modeler;
  modeler.setMinViewsPerSurfel(1);
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeNormals, 1);
  rgbd_processor.setMaxNormalAngle(90);
#elif 0
  ICPSurfelsRGBDModeler modeler;
  modeler.setMinViewsPerSurfel(1);
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeNormals, 1);
  rgbd_processor.setMaxNormalAngle(60);
#elif 0 && defined(NESTK_PRIVATE)
  SilhouetteRGBDModeler modeler;
  modeler.initialize(Point3f(0.5,0.5,0.5), Point3f(-0.25, -0.2, -0.8), 0.01, 0.01);
#else
  RGBDModeler modeler;
  rgbd_processor.setFilterFlag(RGBDProcessor::ComputeNormals, 1);
#endif

  // Pa10ModelAcquisitionController model_acq_controller (gui_controller, modeler);
  ModelAcquisitionController* acq_controller = 0;
  if (opt::pa10())
  {
    if (opt::pose_controller())
      acq_controller = new Pa10PoseAcquisitionController (gui_controller, modeler);
    else
      acq_controller = new Pa10ModelAcquisitionController (gui_controller, modeler);
  }
  else
    acq_controller = new ModelAcquisitionController (gui_controller, modeler);

  RelativePoseEstimator* pose_estimator = 0;
  if (opt::pose_estimator() == std::string("file"))
  {
    pose_estimator = new RelativePoseEstimatorFromFile();
  }
  else if (opt::pose_estimator() == std::string("delta"))
  {
    ntk_ensure(opt::model_initial_pose() && opt::model_delta_pose(),
               "Must specify initial and delta pose if delta pose estimator chosen.");
    Pose3D initial_pose;
    initial_pose.parseAvsFile(opt::model_initial_pose());
    Pose3D delta_pose;
    delta_pose.parseAvsFile(opt::model_delta_pose());    
    pose_estimator = new RelativePoseEstimatorFromDelta(initial_pose, delta_pose);
  }
  else if (opt::pose_estimator() == std::string("image"))
  {
    pose_estimator = new RelativePoseEstimatorFromImage();
  }

  acq_controller->setPoseEstimator(pose_estimator);
  gui_controller.setModelAcquisitionController(*acq_controller);

  if (pa10_client)
    gui_controller.setPa10Client(*pa10_client, initial_angles);
  if (mesh_generator)
    gui_controller.setMeshGenerator(*mesh_generator);

  grabber->start();

  app.exec();
  delete mesh_generator;
  delete acq_controller;

  if (server.isSupported())
    server.stop();
}
