
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

ntk::arg<bool> no_registration("--no-registration", "No color-depth hardware registration.", 0);

ntk::arg<bool> openni("--openni", "Force OpenNI driver", 0);
ntk::arg<bool> freenect("--freenect", "Force freenect driver", 0);
ntk::arg<bool> kin4win("--kin4win", "Force kin4win driver", 0);
ntk::arg<bool> softkinetic("--softkinetic", "Force softkinetic driver", 0);
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
        params.default_type = RGBDGrabberFactory::OPENNI;

    if (opt::freenect())
        params.default_type = RGBDGrabberFactory::FREENECT;

    if (opt::kin4win())
        params.default_type = RGBDGrabberFactory::KIN4WIN;

    if (opt::softkinetic())
        params.default_type = RGBDGrabberFactory::SOFTKINETIC;

    if (opt::pmd())
        params.default_type = RGBDGrabberFactory::PMD;

    if (opt::calibration_dir())
        params.calibration_dir = opt::calibration_dir();

    if (opt::high_resolution())
        params.high_resolution = true;

    if (opt::no_registration())
        params.hardware_registration = false;

    if (opt::sync())
        params.synchronous = true;

    std::vector<RGBDGrabberFactory::GrabberData> grabbers;
    grabbers = grabber_factory.createGrabbers(params);

    ntk_dbg_print(grabbers.size(), 1);

    for (int i = 0; i < grabbers.size(); ++i)
    {
        RGBDGrabber* grabber = grabbers[i].grabber;       
        bool ok = grabber->connectToDevice();
        if (!ok)
        {
            ntk_dbg(0) << "WARNING: connectToDevice failed.";
            continue;
        }

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
