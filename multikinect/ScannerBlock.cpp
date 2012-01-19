#include "ScannerBlock.h"

#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/utils/time.h>
#include <ntk/geometry/relative_pose_estimator_icp.h>

using namespace ntk;

//------------------------------------------------

RecorderBlock::RecorderBlock()
{
    m_recorder = new RGBDFrameRecorder("/tmp/test");
    m_recorder->setSaveOnlyRaw(true);
    m_recorder->setUseBinaryRaw(true);
    m_recorder->setSaveIntensity(false);
    m_recorder->setUseCompressedFormat(false);
}

void RecorderBlock::setOutputDirectoryPrefix(const std::string &dir)
{
    QMutexLocker _(&m_recorder_mutex);
    m_recorder->setDirectory(dir);
}

void RecorderBlock::run()
{
    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent(200);
        if (event.isNull())
            continue;

        RGBDImagePtr image = dynamic_Ptr_cast<RGBDImage>(event.data);
        ntk_assert(image, "No image!");

        {
            QMutexLocker _(&m_recorder_mutex);
            m_recorder->saveCurrentFrame(*image);
        }

        reportNewEventProcessed();
    }
}

//------------------------------------------------

void RGBDProcessorBlock::setProcessorFilterFlag(ntk::RGBDProcessorFlags::Flag flag, bool enabled)
{
    QMutexLocker _(&m_processor_mutex);
    m_processor.setFilterFlag(flag, enabled);
}

void RGBDProcessorBlock::run()
{
    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent(200);
        if (event.isNull())
            continue;

        FrameVectorPtr data = dynamic_Ptr_cast<FrameVector>(event.data);
        ntk_assert(data, "No images!");

        for (size_t i = 0; i < data->images.size(); ++i)
        {
            QMutexLocker _(&m_processor_mutex);
            m_processor.processImage(*data->images[i]);
        }
        broadcastEvent(data);
        // FIXME: make this configurable.
        ntk::sleep(50);

        reportNewEventProcessed();
    }
}

//------------------------------------------------

void FrameSynchronizerBlock::run()
{
    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent(1000);
        if (event.isNull())
            continue;

        RGBDImagePtr image = dynamic_Ptr_cast<RGBDImage>(event.data);
        if (!image)
            continue;

        ntk_dbg(2) << "[FrameSync] New image received";

        // Another device has to be waited for?
        if (m_devices_serial_to_wait.find(image->cameraSerial()) == m_devices_serial_to_wait.end())
            continue;

        // Update the image.
        m_updated_images[image->cameraSerial()] = image;

        // Not all images received? Keep waiting for more events.
        if (m_updated_images.size() < m_devices_serial_to_wait.size())
            continue;

        // Everything received. Can generate a new event.
        FrameVectorPtr new_data (new FrameVector);
        foreach_const_it(it, m_updated_images, image_map_type)
            new_data->images.push_back(it->second);
        broadcastEvent(new_data);

        ntk_dbg(2) << "[FrameSync] All images updated, sending new event!";

        // Reset images.
        m_updated_images.clear();

        reportNewEventProcessed();
    }
}

//------------------------------------------------

MeshGeneratorBlock::MeshGeneratorBlock()
{
    m_mesh_generator.setMaxDeltaDepthBetweenEdges(0.01);
}

void MeshGeneratorBlock::setMeshType(ntk::MeshGenerator::MeshType type)
{
    QMutexLocker _(&m_generator_mutex);
    m_mesh_generator.setMeshType(type);
}

void MeshGeneratorBlock::setMeshUseColor(bool use_it)
{
    QMutexLocker _(&m_generator_mutex);
    m_mesh_generator.setUseColor(use_it);
}

void MeshGeneratorBlock::setMeshResolutionFactor(double value)
{
    QMutexLocker _(&m_generator_mutex);
    m_mesh_generator.setResolutionFactor(value);
}

void MeshGeneratorBlock::run()
{
    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent(1000);
        if (event.isNull())
            continue;

        FrameVectorConstPtr data = dynamic_Ptr_cast<FrameVector>(event.data);
        if (!data)
            continue;

        ntk_dbg(2) << "[MeshGeneratorBlock] New image received";

        MeshVectorPtr new_event (new MeshVector());
        new_event->meshes.resize(data->images.size());
        new_event->camera_serials.resize(data->images.size());

        {
            QMutexLocker _(&m_generator_mutex);
            for (size_t i = 0; i < data->images.size(); ++i)
            {
                m_mesh_generator.generate(*data->images[i]);
                MeshPtr mesh (new Mesh);
                *mesh = m_mesh_generator.mesh();
                new_event->meshes[i] = mesh;
                new_event->camera_serials[i] = data->images[i]->cameraSerial();
            }
        }

        broadcastEvent(new_event);

        ntk_dbg(2) << "[MeshGeneratorBlock] All meshes created, sending new event!";

        reportNewEventProcessed();
    }
}

//------------------------------------------------

void CalibratorBlock::setCalibrationPattern(float square_size, int num_corners_x, int num_corners_y)
{
    QMutexLocker _(&m_calibrator_mutex);
    m_pattern_size = square_size;
    m_pattern_num_corners_x = num_corners_x;
    m_pattern_num_corners_y = num_corners_y;
}

void CalibratorBlock::setCalibrationAlgorithm(CalibratorBlock::Algorithm algo)
{
    QMutexLocker _(&m_calibrator_mutex);
    m_algo = algo;
}

void CalibratorBlock::calibrateWithICP(FrameVectorConstPtr frames)
{
    if (frames->images.size() < 2)
    {
        ntk_dbg(0) << "Only one device, no need for alignment!";
        return;
    }

    // Compute Normals
    std::vector<RGBDImage> images (frames->images.size());
    OpenniRGBDProcessor processor;
    foreach_idx(i, images)
    {
        if (!frames->images[i]->calibration())
            continue;
        frames->images[i]->copyTo(images[i]);
        processor.computeNormalsPCL(images[i]);
    }

    if (!images[0].calibration())
    {
        ntk_dbg(0) << "The first camera " << frames->images[0]->cameraSerial()
                   << " does not have calibration. It can´t be the reference, returning.";
        return;
    }

    for (size_t i = 1; i < images.size(); ++i)
    {
        if (!images[i].calibration())
        {
            ntk_dbg(0) << "Warning: camera " << images[i].cameraSerial() << " does not have calibration.";
            continue;
        }

        RelativePoseEstimatorICPWithNormals<pcl::PointNormal> estimator;
        estimator.setDistanceThreshold(0.05);
        estimator.setVoxelSize(0.01);
        estimator.setTargetImage(images[0]);
        estimator.setSourceImage(images[i]);
        bool ok = estimator.estimateNewPose();
        if (!ok)
            continue;

        Pose3D new_pose = *images[i].calibration()->depth_pose;
        new_pose.applyTransformBefore(estimator.estimatedSourcePose());

        CalibrationParametersPtr params (new CalibrationParameters);
        params->new_t = new_pose.cvTranslation();
        params->new_r = new_pose.cvEulerRotation();
        params->camera_serial = images[i].cameraSerial();
        params->calibration = images[i].calibration();
        broadcastEvent(params);
    }
}

void CalibratorBlock::calibrateWithChessboard(FrameVectorConstPtr frames)
{
    for (size_t i = 0; i < frames->images.size(); ++i)
    {
        if (!frames->images[i]->calibration())
        {
            ntk_dbg(0) << "Warning: camera " << frames->images[i]->cameraSerial() << " does not have calibration.";
            continue;
        }

        std::vector<cv::Point3f> model;
        calibrationPattern(model, m_pattern_num_corners_x, m_pattern_num_corners_y, m_pattern_size);

        std::vector<cv::Point2f> corners;
        calibrationCorners(cv::format("debug_%d", i), "",
                           m_pattern_num_corners_x, m_pattern_num_corners_y,
                           corners,
                           frames->images[i]->rgb(),
                           1.0);

        if (corners.size() == 0)
        {
            ntk_dbg(0) << "Warning: chessboard not detected in image " << i;
            continue;
        }

        const RGBDCalibration* calibration = frames->images[i]->calibration();

        cv::Mat1f H;
        estimate_checkerboard_pose(model, corners, calibration->rgb_intrinsics, H);

        // Set the camera aligned with the chessboard, looking at it.
        // The new origin will be the first corner of the board.
        Pose3D H_pose;
        H_pose.setCameraTransform(H);
        H_pose.toLeftCamera(calibration->depth_intrinsics, calibration->R, calibration->T);
        H_pose.applyTransformBefore(0, cv::Vec3f(M_PI,0,0));

        CalibrationParametersPtr params (new CalibrationParameters);
        params->new_t = H_pose.cvTranslation();
        params->new_r = H_pose.cvEulerRotation();
        ntk_dbg_print(params->new_t, 0);
        ntk_dbg_print(params->new_r, 0);
        params->camera_serial = frames->images[i]->cameraSerial();
        params->calibration = frames->images[i]->calibration();
        broadcastEvent(params);
    }
}

void CalibratorBlock::run()
{
    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent(1000);
        if (event.isNull())
            continue;

        FrameVectorConstPtr data = dynamic_Ptr_cast<FrameVector>(event.data);
        if (!data)
            continue;

        ntk_dbg(1) << "[CalibratorBlock] New image received";

        {
            QMutexLocker _(&m_calibrator_mutex);
            switch (m_algo)
            {
            case ICP:
                calibrateWithICP(data);
                break;
            case Chessboard:
                calibrateWithChessboard(data);
                break;
            default:
                ;
            };
        }

        ntk_dbg(1) << "[CalibratorBlock] Calibration finished.";
        reportNewEventProcessed();
    }
}
