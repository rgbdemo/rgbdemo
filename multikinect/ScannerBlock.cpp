#include "ScannerBlock.h"

#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/utils/time.h>
#include <ntk/geometry/relative_pose_estimator_icp.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>

using namespace ntk;

//------------------------------------------------

RecorderBlock::RecorderBlock(Name name)
: ScannerBlock(name)
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
        EventListener::Event event = waitForNewEvent();
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
        EventListener::Event event = waitForNewEvent();
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

bool FrameSynchronizerBlock::otherImagesHaveNoDepth(const std::string& serial) const
{
    const int64 min_delay_msecs = 200;

    if (m_images_without_depth.size() != m_devices_serial_to_wait.size())
        return false;

    uint64 current = ntk::Time::getMillisecondCounter();

    foreach_const_it(it, m_images_without_depth, images_without_depth_map_type)
    {
        if (it->first == serial)
            continue;
        if ((it->second == 0) || ((current - it->second) < min_delay_msecs))
            return false;
    }

    return true;
}

void FrameSynchronizerBlock::setWaitUntilOnlyOneFrameHasDepthMode(bool enable)
{
    QMutexLocker _(&mutex);
    m_wait_until_only_one_has_depth = enable;
    m_images_without_depth.clear();
    m_last_added_image.clear();
    m_updated_images.clear();
}

void FrameSynchronizerBlock::run()
{
    // m_wait_until_only_one_has_depth = true;

    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent();
        if (event.isNull())
            continue;

        RGBDImagePtr image = dynamic_Ptr_cast<RGBDImage>(event.data);
        if (!image)
            continue;

        ntk_dbg(2) << "[FrameSync] New image received";

        // Another device has to be waited for?
        if (m_devices_serial_to_wait.find(image->cameraSerial()) == m_devices_serial_to_wait.end())
            continue;

        QMutexLocker _(&mutex);

        if (m_wait_until_only_one_has_depth)
        {
            if (image->hasEmptyRawDepthImage())
            {
                images_without_depth_map_type::iterator it;
                it = m_images_without_depth.find(image->cameraSerial());
                if (it == m_images_without_depth.end() || it->second == 0)
                    m_images_without_depth[image->cameraSerial()] = ntk::Time::getMillisecondCounter();
                continue;
            }

            m_images_without_depth[image->cameraSerial()] = 0;
            if (!otherImagesHaveNoDepth(image->cameraSerial()))
                continue;
        }

        // Require an alternative activation of each camera.
        if (m_wait_until_only_one_has_depth
                && m_last_added_image == image->cameraSerial())
            continue;

        // Update the image.
        m_updated_images[image->cameraSerial()] = image;
        m_last_added_image = image->cameraSerial();
        ntk_dbg(2) << image->cameraSerial() << " validated";

        // Not all images received? Keep waiting for more events.
        if (m_updated_images.size() < m_devices_serial_to_wait.size())
            continue;

        // Everything received. Can generate a new event.
        FrameVectorPtr new_data (new FrameVector);
        new_data->images.resize(m_updated_images.size());
        int i = 0;
        foreach_const_it(it, m_updated_images, image_map_type)
            new_data->images[i++] = it->second;
        broadcastEvent(new_data);

        ntk_dbg(2) << "[FrameSync] All images updated, sending new event!";

        // Reset images.
        m_updated_images.clear();

        if (m_wait_until_only_one_has_depth)
            m_images_without_depth.clear();

        reportNewEventProcessed();
    }
}

//------------------------------------------------

MeshGeneratorBlock::MeshGeneratorBlock(Name name)
: ScannerBlock(name)
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
        EventListener::Event event = waitForNewEvent();
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
    m_pattern_width = num_corners_x;
    m_pattern_height = num_corners_y;
}

void CalibratorBlock::setCalibrationAlgorithm(CalibratorBlock::Algorithm algo)
{
    QMutexLocker _(&m_calibrator_mutex);
    m_algo = algo;
}

void CalibratorBlock::calibrateWithICP(FrameVectorVectorConstPtr frames_)
{
    FrameVectorConstPtr frames = frames_->frames[0];
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

namespace {

struct pose_error_3d : public ntk::CostFunction
{
    pose_error_3d(const Pose3D& initial_pose,
                          const std::vector<cv::Point3f>& ref_points,
                          const std::vector<cv::Point3f>& img_points)
        : CostFunction(6, ref_points.size()*3),
          initial_pose(initial_pose),
          ref_points(ref_points),
          img_points(img_points)
    {
        ntk_assert(ref_points.size() == img_points.size(), "Invalid matches");
    }

    virtual void evaluate (const std::vector< double > &x, std::vector< double > &fx) const
    {
        Pose3D new_pose = initial_pose;
        new_pose.applyTransformAfter(cv::Vec3f(x[3],x[4],x[5]), cv::Vec3f(x[0],x[1],x[2]));
        int err_i = 0;
        std::fill(stl_bounds(fx), 0);
        foreach_idx(p_i, ref_points)
        {
            const cv::Point3f& ref_point = ref_points[p_i];
            const cv::Point3f& img_point = img_points[p_i];
            cv::Point3f p3d = new_pose.unprojectFromImage(img_point);

            fx[err_i*3] = (p3d.x-ref_point.x);
            fx[err_i*3+1] = (p3d.y-ref_point.y);
            fx[err_i*3+2] = (p3d.z-ref_point.z);

            err_i = err_i + 1;
        }
    }

private:
    const Pose3D& initial_pose;
    const std::vector<cv::Point3f>& ref_points;
    const std::vector<cv::Point3f>& img_points;
};

// atomic mean square pose estimation.
double refinePoseUsingDepth(Pose3D& pose3d,
                            const std::vector<cv::Point3f>& ref_points,
                            const std::vector<cv::Point3f>& img_points)
{
    std::vector<double> fx;
    std::vector<double> initial(6);
    pose_error_3d f(pose3d, ref_points, img_points);
    LevenbergMarquartMinimizer optimizer;
    std::fill(stl_bounds(initial), 0);
    fx.resize(ref_points.size()*3);

    ntk_dbg_print(f.normalizedOutputNorm(initial), 1);

    optimizer.minimize(f, initial);
    optimizer.diagnoseOutcome(1);
    f.evaluate(initial, fx);
    foreach_idx(i, fx)
    {
        ntk_dbg_print(fx[i], 1);
    }

    // FIXME: use normalized norm?
    double error = f.normalizedOutputNorm(initial);

    pose3d.applyTransformAfter(cv::Vec3f(initial[3],initial[4],initial[5]), cv::Vec3f(initial[0], initial[1], initial[2]));
    ntk_dbg_print(error, 1);
    return error;
}

} // anonymous

void CalibratorBlock::calibrateWithChessboard(FrameVectorVectorConstPtr frames)
{
    std::vector<RGBDImage> ref_images;

    if (frames->frames.size() == 0 || frames->frames[0]->images.size() < 2)
    {
        ntk_dbg(0) << "Calibration required but no valid vector of images.";
        return;
    }

    for (int i = 0; i < frames->frames.size(); ++i)
    {
        ref_images.push_back(*frames->frames[i]->images[0]);
    }

    std::vector< std::vector<cv::Point2f> > ref_corners, ref_good_corners;
    getCalibratedCheckerboardCorners(ref_images,
                                     m_pattern_width, m_pattern_height, PatternChessboard,
                                     ref_corners,
                                     ref_good_corners,
                                     false /* show corners */);

    if (ref_good_corners.size() != ref_images.size())
    {
        ntk_dbg(0) << "Could not extract corners from the first image.";
        return;
    }

    int num_cameras = frames->frames[0]->images.size();
    for (int i_camera = 1; i_camera < num_cameras; ++i_camera)
    {
        std::vector<RGBDImage> images;
        for (int i = 0; i < frames->frames.size(); ++i)
        {
            images.push_back(*frames->frames[i]->images[i_camera]);
        }

        ntk_dbg_print(images.size(), 1);

        std::vector< std::vector<cv::Point2f> > corners, good_corners;
        getCalibratedCheckerboardCorners(images,
                                         m_pattern_width, m_pattern_height, PatternChessboard,
                                         corners,
                                         good_corners,
                                         false /* show corners */);

        if (corners.size() == 0)
            continue;

        std::vector<cv::Point3f> ref_3d_points;
        std::vector<cv::Point3f> target_img_points;
        foreach_idx(i, ref_corners)
        {
            if (ref_corners[i].size() == 0 || corners[i].size() == 0)
                continue;

            foreach_idx(j, ref_corners[i])
            {
                cv::Point2f pixel_ref = ref_corners[i][j];
                if (!ref_images[i].rgbPixelHasDepth(pixel_ref.y, pixel_ref.x))
                    continue;

                cv::Point2f pixel_new = corners[i][j];
                if (!images[i].rgbPixelHasDepth(pixel_new.y, pixel_new.x))
                    continue;

                const float ref_depth = ref_images[i].mappedDepth()(pixel_ref.y, pixel_ref.x);
                ref_3d_points.push_back(ref_images[0].calibration()->depth_pose->unprojectFromImage(pixel_ref, ref_depth));

                const float target_depth = images[i].mappedDepth()(pixel_new.y, pixel_new.x);
                target_img_points.push_back(cv::Point3f(pixel_new.x, pixel_new.y, target_depth));
            }
        }

        ntk_ensure(images[0].calibration(), "Images are not calibrated, cannot compute extrinsics!");
        RGBDCalibration& calibration = *const_cast<RGBDCalibration*>(images[0].calibration());
        // images[0].calibration()->copyTo(calibration);

        ntk_dbg_print(m_pattern_size, 1);

        calibrateStereoFromCheckerboard(ref_corners, corners,
                                        m_pattern_width, m_pattern_height, m_pattern_size,
                                        calibration);        

        calibration.updatePoses();

        calibration.depth_pose->applyTransformBefore(*ref_images[0].calibration()->depth_pose);
        calibration.depth_pose->cvRotationMatrixTranslation(calibration.T_extrinsics, calibration.R_extrinsics);
        calibration.updatePoses();

        if (false && ref_3d_points.size() > 10)
        {
            refinePoseUsingDepth(*calibration.depth_pose, ref_3d_points, target_img_points);
            calibration.depth_pose->cvRotationMatrixTranslation(calibration.T_extrinsics, calibration.R_extrinsics);
            calibration.updatePoses(); // FIXME: necessary?
        }

        cv::Mat3b debug_img;
        images[0].rgb().copyTo(debug_img);

        for (int i = 0; i < ref_corners[0].size(); ++i)
        {
            cv::Point2f p = ref_corners[0][i];
            float d = ref_images[0].mappedDepth()(p.y, p.x);
            if (d < 1e-5) continue;
            cv::Point3f p3d = ref_images[0].calibration()->rgb_pose->unprojectFromImage(cv::Point3f(p.x, p.y, d));

            cv::Point3f pp = images[0].calibration()->rgb_pose->projectToImage(p3d);
            cv::circle(debug_img, cv::Point2i(pp.x, pp.y), 5, cv::Scalar(255,255,0));
        }
        cv::imwrite("debug_reprojected.png", debug_img);

#if 0
        CalibrationParametersPtr params (new CalibrationParameters);
        params->new_t = calibration.depth_pose->cvTranslation();
        params->new_r = calibration.depth_pose->cvEulerRotation();
        ntk_dbg_print(params->new_t, 0);
        ntk_dbg_print(params->new_r, 0);
        params->camera_serial = images[0].cameraSerial();
        params->calibration = images[0].calibration();
        broadcastEvent(params);
#endif
    }
}

#if 0
void CalibratorBlock::calibrateWithChessboard(FrameVectorVectorConstPtr frames_)
{
    FrameVectorConstPtr frames = frames_->frames[0];
    for (size_t i = 0; i < frames->images.size(); ++i)
    {
        if (!frames->images[i]->calibration())
        {
            ntk_dbg(0) << "Warning: camera " << frames->images[i]->cameraSerial() << " does not have calibration.";
            continue;
        }

        std::vector<cv::Point3f> model;
        calibrationPattern(model, m_pattern_width, m_pattern_height, m_pattern_size);

        std::vector<cv::Point2f> corners;
        calibrationCorners(cv::format("debug_%d", i), "",
                           m_pattern_width, m_pattern_height,
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
#endif

void CalibratorBlock::run()
{
    while (!threadShouldExit())
    {
        EventListener::Event event = waitForNewEvent();
        if (event.isNull())
            continue;

        FrameVectorVectorConstPtr data = dynamic_Ptr_cast<FrameVectorVector>(event.data);
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
