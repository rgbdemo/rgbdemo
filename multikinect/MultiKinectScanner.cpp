#include "MultiKinectScanner.h"

#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/utils/time.h>

#include <ntk/mesh/mesh_generator.h>

#include <QApplication>

using namespace ntk;

MultiKinectScanner::MultiKinectScanner()
    : m_controller(0),
      m_paused(false)
{
    m_processor_block.addEventListener(this);

    m_frame_synchronizer_block.addEventListener(this);

    m_mesh_generator_block.addEventListener(this);

    m_recorder_block.addEventListener(this);
    m_recorder_block.setConnected(false); // disconnected by default.

    m_calibrator_block.addEventListener(this);
}

bool MultiKinectScanner::initialize()
{
    bool ok = true;
    foreach_it(it, m_grabbers, std::set<RGBDGrabber*>)
    {
        ok &= (*it)->connectToDevice();
        DeviceInfo info;
        info.serial = (*it)->cameraSerial();
        info.calibration = (*it)->calibrationData();
        m_devices.push_back(info);
    }
    return ok;
}

void MultiKinectScanner::addGrabber(RGBDGrabber *grabber)
{    
    m_grabbers.insert(grabber);
    grabber->addEventListener(this);
}

void MultiKinectScanner::triggerGrabbers()
{
    foreach_const_it(it, m_grabbers, std::set<ntk::RGBDGrabber*>)
    {
        (*it)->newEvent();
    }
}

void MultiKinectScanner::setGrabberSynchronous(bool sync)
{
    foreach_const_it(it, m_grabbers, std::set<ntk::RGBDGrabber*>)
    {
        (*it)->setSynchronous(sync);
        (*it)->newEvent();
    }
}

bool MultiKinectScanner::areGrabbersSynchronous() const
{
    bool sync = true;
    foreach_const_it(it, m_grabbers, std::set<ntk::RGBDGrabber*>)
    {
        sync &= (*it)->isSynchronous();
    }
    return sync;
}

void MultiKinectScanner::setPaused(bool paused)
{
    m_paused = paused;
    triggerGrabbers();
}

void MultiKinectScanner::processOneFrame()
{
    m_process_one_frame = true;
    triggerGrabbers();
}

void MultiKinectScanner::saveGrabbersCalibration(const std::string &prefix)
{
    foreach_const_it(it, m_grabbers, std::set<ntk::RGBDGrabber*>)
    {
        const RGBDCalibration* calibration = (*it)->calibrationData();
        std::string serial = (*it)->cameraSerial();
        if (!calibration)
        {
            ntk_dbg(0) << "Warning: " << serial << " does not have calibration";
            continue;
        }
        calibration->saveToFile(cv::format("%s-%s.yml", prefix.c_str(), serial.c_str()).c_str());
    }
}

void MultiKinectScanner::processLastImageFromGrabber(RGBDGrabber& grabber)
{
    RGBDImagePtr image (new RGBDImage());
    grabber.copyImageTo(*image);

    m_recorder_block.newEvent(this, EventDataPtr(image));
    m_frame_synchronizer_block.newEvent(this, (EventDataPtr)image);
}

void MultiKinectScanner::calibrateCameras()
{
    if (m_last_processed_frame_vector)
        m_calibrator_block.newEvent(this, m_last_processed_frame_vector);
}

void MultiKinectScanner::updateCameraCalibration(CalibrationParametersPtr params)
{
    ntk_dbg(1) << "[MultiKinectScanner] New calibration parameters received";

    // FIXME: should be using a more complex mechanism here to avoid race conditions.
    // FIXME: there is a strong need for Mutexes and redesign here since the
    // calibration pointer is shared everywhere. Modifying it is dangerous.
    RGBDCalibration* calibration = const_cast<RGBDCalibration*>(params->calibration);

    Pose3D p;
    p.applyTransformBefore(params->new_t, params->new_r);
    p.cvRotationMatrixTranslation(calibration->T_extrinsics, calibration->R_extrinsics);
    calibration->updatePoses();

    if (m_controller)
        m_controller->onCameraExtrinsicsChanged(params);
}

void MultiKinectScanner::run()
{
    initialize();
    if (m_controller)
        m_controller->onScannerInitialized();

    setPriority(QThread::HighestPriority);

    m_recorder_block.start();
    m_recorder_block.setPriority(QThread::HighPriority);

    // m_processor_block.setProcessorFilterFlag(RGBDProcessorFlags::FilterBilateral, true);
    // m_processor_block.setProcessorFilterFlag(RGBDProcessorFlags::ComputeNormals, true);
    m_processor_block.start();
    m_processor_block.setPriority(QThread::LowestPriority);

    for (size_t i = 0; i < m_devices.size(); ++i)
        m_frame_synchronizer_block.addDevice(m_devices[i].serial);
    m_frame_synchronizer_block.start();
    m_frame_synchronizer_block.setPriority(QThread::LowestPriority);

    m_mesh_generator_block.start();
    m_mesh_generator_block.setPriority(QThread::LowestPriority);

    m_calibrator_block.start();
    m_calibrator_block.setPriority(QThread::LowestPriority);

    foreach_it(it, m_grabbers, std::set<RGBDGrabber*>)
    {
        (*it)->start();
    }

    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent(1000);
        if (event.isNull())
            continue;

        // NOTE: some blocks could be connected automatically without requiring to go
        // through the main scanner loop. The intent here is to make sure that, if desired,
        // it is still possible to manage all the interactions from a single "master" location.

        // Comes from a grabber ?
        ntk::RGBDGrabber* grabber = dynamic_cast<ntk::RGBDGrabber*>(event.sender);
        if (grabber != 0)
        {
            if (m_paused && !m_process_one_frame)
                continue;

            if (m_process_one_frame)
            {
                m_process_one_frame = false;
            }

            ntk_assert(m_grabbers.find(grabber) != m_grabbers.end(), "Receiving messages from unknown grabber");
            processLastImageFromGrabber(*grabber);
        }

        // Frame synchronizer block output.
        else if (event.sender == (EventBroadcaster*)(&m_frame_synchronizer_block) && m_controller)
        {
            FrameVectorPtr data = dynamic_Ptr_cast<FrameVector>(event.data);
            ntk_assert(data, "Inconsistent data type");
            m_processor_block.newEvent(this, data);
        }

        // Processor block output
        else if (event.sender == (EventBroadcaster*)(&m_processor_block) && m_controller)
        {
            FrameVectorPtr data = dynamic_Ptr_cast<FrameVector>(event.data);
            ntk_assert(data, "Inconsistent data type");
            m_mesh_generator_block.newEvent(this, data);
            m_controller->onNewSynchronizedImages(data->images);
            m_last_processed_frame_vector = data;
            triggerGrabbers();
        }

        // Mesh generator block output.
        else if (event.sender == (EventBroadcaster*)(&m_mesh_generator_block) && m_controller)
        {
            MeshVectorPtr data = dynamic_Ptr_cast<MeshVector>(event.data);
            ntk_assert(data, "Inconsistent data type");
            m_controller->onNewSynchronizedMeshes(data);
        }

        // Calibrator block output
        else if (event.sender == (EventBroadcaster*)(&m_calibrator_block) && m_controller)
        {
            CalibrationParametersPtr params = dynamic_Ptr_cast<CalibrationParameters>(event.data);
            ntk_assert(params, "Inconsistent data type");
            updateCameraCalibration(params); // running synchronously. this is not a good idea.
        }

        reportNewEventProcessed();
    }

    m_recorder_block.setThreadShouldExit();
    m_processor_block.setThreadShouldExit();
    m_frame_synchronizer_block.setThreadShouldExit();
    m_calibrator_block.setThreadShouldExit();
    m_mesh_generator_block.setThreadShouldExit();

    m_recorder_block.wait();
    m_processor_block.wait();    
    m_frame_synchronizer_block.wait();
    m_calibrator_block.wait();
    m_mesh_generator_block.wait();

    foreach_const_it(it, m_grabbers, std::set<ntk::RGBDGrabber*>)
    {
        (*it)->stop();
        (*it)->disconnectFromDevice();
    }
}

//------------------------------------------------

void MultiKinectController::quit()
{
    m_scanner->setThreadShouldExit();
    m_scanner->wait();
    QApplication::quit();
}


