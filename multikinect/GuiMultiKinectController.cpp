#include "GuiMultiKinectController.h"

#include "RawImagesWindow.h"
#include "ui_RawImagesWindow.h"

#include "View3dWindow.h"
#include "ui_View3dWindow.h"

#include "FiltersWindow.h"

#include <ntk/utils/time.h>
#include <ntk/camera/calibration.h>

using namespace ntk;

GuiMultiKinectController::GuiMultiKinectController(MultiKinectScanner* scanner)
    : MultiKinectController(scanner),
      m_grabbing(false),
      m_checkerboard_frames(new FrameVectorVector),
      m_frame_recorder("")
{
    addEventListener(this);
    m_init_broadcaster.addEventListener(this);
    m_synchronized_images_broadcaster.addEventListener(this);
    m_synchronized_meshes_broadcaster.addEventListener(this);

    m_raw_images_window = new RawImagesWindow(*this);
    m_filters_window = new FiltersWindow(*this);
    m_view3d_window = new View3DWindow(*this);

    m_raw_images_window->show();

    this->scanner().meshGeneratorBlock().setConnected(false);
    this->scanner().meshGeneratorBlock().setMeshUseColor(m_view3d_window->ui->colorMappingCheckBox->isChecked());

    setMergeViews(m_view3d_window->ui->mergeViewsCheckBox->isChecked());

    m_frame_recorder.setDirectory(m_raw_images_window->ui->outputDirText->text().toStdString());
    m_frame_recorder.setSaveOnlyRaw(true);
    m_frame_recorder.setUseBinaryRaw(true);
}

GuiMultiKinectController::~GuiMultiKinectController()
{
    delete m_raw_images_window;
}

void GuiMultiKinectController::onNewImage(RGBDImageConstPtr image)
{
    LocalEventDataPtr data (new LocalEventData);
    data->new_image = image;
    broadcastEvent(data);
}

void GuiMultiKinectController::onNewSynchronizedImages(const std::vector<ntk::RGBDImagePtr> &images)
{
    LocalEventDataPtr data (new LocalEventData);
    data->new_synchronized_images = images;
    m_synchronized_images_broadcaster.broadcastEvent(data);
}

void GuiMultiKinectController::onNewSynchronizedMeshes(MeshVectorPtr meshes)
{
    LocalEventDataPtr data (new LocalEventData);
    data->new_synchronized_meshes = meshes;
    m_synchronized_meshes_broadcaster.broadcastEvent(data);
}

void GuiMultiKinectController::onScannerInitialized()
{
    LocalEventDataPtr data (new LocalEventData);
    data->event_type = LocalEventData::ScannerInitialized;
    // Use a different broadcaster here to make sure that this event
    // does not get overridden by other kind of events.
    m_init_broadcaster.broadcastEvent(data);
}

void GuiMultiKinectController::handleAsyncEvent(ntk::EventListener::Event event)
{        
    // Now in GUI thread.  
    LocalEventDataPtr data = dynamic_Ptr_cast<LocalEventData>(event.data);

    if (data->event_type == LocalEventData::ScannerInitialized)
        handleScannerInitialized();

    if (data->new_image)
        processNewImage(data->new_image);

    if (data->new_synchronized_images.size() > 0)
        processNewSynchronizedImages(data->new_synchronized_images);

    if (data->new_synchronized_meshes)
        processNewSynchronizedMeshes(data->new_synchronized_meshes);

    if (data->calibration_parameters)
        processNewCalibrationParameters(data->calibration_parameters);
}

void GuiMultiKinectController::processNewImage(RGBDImageConstPtr image)
{
    if (image->cameraSerial() != m_active_device_serial)
        return;

    image->copyTo(m_last_image);
    if (m_raw_images_window->isVisible())
        m_raw_images_window->update(*image);

    QString status;
    if (m_grabbing)
    {
        status = QString("[GRABBING] Processor = %1 fps / Recorder = %2 fps / MainLoop = %3 fps")
                .arg(scanner().processorBlock().frameRate(), 0, 'f', 1)
                .arg(scanner().recorderBlock().frameRate(), 0, 'f', 1)
                .arg(scanner().frameRate(), 0, 'f', 1);
    }
    else
    {
        status = QString("Processor = %1 fps / MainLoop = %2 fps")
                .arg(scanner().processorBlock().frameRate(), 0, 'f', 1)
                .arg(scanner().frameRate(), 0, 'f', 1);
    }

    m_raw_images_window->ui->statusbar->showMessage(status);
}

void GuiMultiKinectController::processNewSynchronizedImages(const std::vector<ntk::RGBDImagePtr> &images)
{
    for (size_t i = 0; i < images.size(); ++i)
        processNewImage(images[i]);
}

void GuiMultiKinectController::processNewSynchronizedMeshes(MeshVectorPtr mesh_vector)
{
    ntk_dbg(2) << "[GuiMultiKinectController] new Synchronized meshes received!";
    m_last_mesh_vector = mesh_vector;

    if (!m_view3d_window->isVisible())
        return;

    for (size_t i = 0; i < mesh_vector->meshes.size(); ++i)
    {
        if (m_merge_views || mesh_vector->camera_serials[i] == m_active_device_serial)
        {
            ntk::Mesh* mesh = mesh_vector->meshes[i];
            m_view3d_window->ui->mesh_view->addMesh(*mesh_vector->meshes[i], Pose3D(), MeshViewer::FLAT);
        }
    }

    ntk::Mesh cube;
    cube.addCube(m_bounding_box.centroid(), cv::Point3f(m_bounding_box.width, m_bounding_box.height, m_bounding_box.depth));
    m_view3d_window->ui->mesh_view->addMesh(cube, Pose3D(), MeshViewer::WIREFRAME);

    m_view3d_window->ui->mesh_view->swapScene();
}

void GuiMultiKinectController :: on_depth_mouse_moved(int x, int y)
{
    if (!m_last_image.depth().data || x < 0 || y < 0)
        return;
    QString s = QString("Distance at (%1,%2) = %3 m")
            .arg(x).arg(y).arg(m_last_image.depth()(y,x), 0, 'f', 3);
    m_raw_images_window->ui->distanceLabel->setText(s);
}

void GuiMultiKinectController::setActiveDevice(int device)
{
    if (device >= scanner().numDevices())
    {
        ntk_dbg(0) << "Warning: no device " << device;
        return;
    }

    ntk_dbg_print(scanner().getDeviceInfo(device).serial, 1);
    m_active_device_serial = scanner().getDeviceInfo(device).serial;
    const RGBDCalibration* calibration = scanner().getDeviceInfo(device).calibration;
    if (calibration)
        m_view3d_window->updateFromCalibration(calibration->depth_pose->cvTranslation(),
                                               calibration->depth_pose->cvEulerRotation());
}

void GuiMultiKinectController::handleScannerInitialized()
{
    ntk_dbg(0) << "handle Scanner initialized";
    setActiveDevice(0);
    if (scanner().canGenerateMeshes())
        m_raw_images_window->ui->action_3D_View->setEnabled(true);
}

void GuiMultiKinectController::toggleFilters(bool active)
{
    m_raw_images_window->ui->action_Filters->setChecked(active);
    if (active)
    {
        m_filters_window->show();
    }
    else
    {
        m_filters_window->hide();
    }
}

void GuiMultiKinectController::toggleView3d(bool active)
{
    m_raw_images_window->ui->action_3D_View->setChecked(active);
    if (active)
    {
        m_view3d_window->show();
        scanner().meshGeneratorBlock().setConnected(true);
    }
    else
    {
        m_view3d_window->hide();
        scanner().meshGeneratorBlock().setConnected(false);
    }
}

void GuiMultiKinectController::saveLastMeshes()
{
    if (!m_last_mesh_vector)
    {
        ntk_dbg(0) << "No meshes to save!";
        return;
    }

    ntk::Mesh global_mesh;
    for (size_t i = 0; i < m_last_mesh_vector->meshes.size(); ++i)
    {
        global_mesh.addMesh(*m_last_mesh_vector->meshes[i]);
        m_last_mesh_vector->meshes[i]->saveToPlyFile(cv::format("current_mesh_%02d.ply", i).c_str());
    }
    global_mesh.saveToPlyFile("current_mesh_global.ply");
}

void GuiMultiKinectController::updateCameraCalibrationToGrabber(const cv::Vec3f &new_t, const cv::Vec3f &new_r)
{
    if (m_active_device_serial != m_last_image.cameraSerial())
        return;

    if (!m_last_image.calibration())
    {
        ntk_dbg(0) << "Warning: last image does not have calibration.";
        return;
    }

    CalibrationParametersPtr params (new CalibrationParameters);
    params->new_t = new_t;
    params->new_r = new_r;
    params->camera_serial = m_last_image.cameraSerial();
    params->calibration = m_last_image.calibration();
    scanner().updateCameraCalibration(params);
}

void GuiMultiKinectController::onCameraExtrinsicsChanged(CalibrationParametersPtr params)
{
    LocalEventDataPtr data (new LocalEventData);
    data->calibration_parameters = params;
    broadcastEvent(data);
}

void GuiMultiKinectController::processNewCalibrationParameters(CalibrationParametersPtr params)
{
    ntk_dbg(1) << "[GuiMultiKinectController]: new parameters received";

    if (m_active_device_serial != params->camera_serial)
        return;

    ntk_dbg(1) << "[GuiMultiKinectController]: updating view3d params";

    m_view3d_window->updateFromCalibration(params->new_t, params->new_r);
}

void GuiMultiKinectController::refineCalibrationWithICP()
{
    scanner().calibratorBlock().setCalibrationAlgorithm(CalibratorBlock::ICP);
    FrameVectorVectorPtr frames (new FrameVectorVector);
    frames->frames.push_back(scanner().lastProcessedFrameVector());
    scanner().calibrateCameras(frames);
}

void GuiMultiKinectController::refineCalibrationWithChessboard()
{    
    int pattern_width = m_view3d_window->ui->patternWidthSpinBox->value();
    int pattern_height = m_view3d_window->ui->patternHeightSpinBox->value();
    float pattern_size = m_view3d_window->ui->patternSizeSpinBox->value();

    scanner().calibratorBlock().setCalibrationAlgorithm(CalibratorBlock::Chessboard);
    scanner().calibratorBlock().setCalibrationPattern(pattern_size, pattern_width, pattern_height);
    FrameVectorVectorPtr frames (new FrameVectorVector);
    frames->frames = m_checkerboard_frames->frames; // keep a copy.
    if (frames->frames.size() == 0)
        return;
    scanner().calibrateCameras(frames);
}

void GuiMultiKinectController::setGrabbing(bool grab)
{
    m_grabbing = grab;
    scanner().recorderBlock().setConnected(grab);
}

void GuiMultiKinectController::grabOneFrame()
{
    FrameVectorPtr frames = scanner().lastProcessedFrameVector();
    std::vector<RGBDImage> images;
    for (int i = 0; i < frames->images.size(); ++i)
    {
        images.push_back(*frames->images[i]);
    }
    m_frame_recorder.saveCurrentFrames(images);
}

void GuiMultiKinectController::setBoundingBox(const Rect3f &bbox, bool update_gui)
{
    m_bounding_box = bbox;
    if (update_gui)
        m_view3d_window->setBoundingBox(bbox);
}

void GuiMultiKinectController::addCheckboardImage()
{
    FrameVectorPtr frame = scanner().lastProcessedFrameVector();
    if (!frame)
        return;

    int pattern_width = m_view3d_window->ui->patternWidthSpinBox->value();
    int pattern_height = m_view3d_window->ui->patternHeightSpinBox->value();
    float pattern_size = m_view3d_window->ui->patternSizeSpinBox->value();

    std::vector<cv::Point2f> corners;
    cv::Mat3b checkerboard_image;
    calibrationCorners("checkboard", "",
                       pattern_width, pattern_height,
                       corners,
                       frame->images[0]->rgb(),
                       1.0,
                       PatternChessboard,
                       &checkerboard_image);
    m_view3d_window->ui->checkerboard_view->setImage(checkerboard_image);
    m_checkerboard_frames->frames.push_back(frame);
    m_view3d_window->ui->labelCheckboard->setText(QString("%1 images").arg(m_checkerboard_frames->frames.size()));
}

void GuiMultiKinectController::resetCheckboardImages()
{
    m_checkerboard_frames = toPtr(new FrameVectorVector);
    m_view3d_window->ui->labelCheckboard->setText(QString("%1 images").arg(m_checkerboard_frames->frames.size()));
}
