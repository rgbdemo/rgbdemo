/**
 * This file is part of the nestk library.
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
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "GuiController.h"
#include "ui_View3dWindow.h"
#include "ui_RawImagesWindow.h"
#include "ui_ModelAcquisitionWindow.h"
#include "View3dWindow.h"
#include "FiltersWindow.h"
#include "RawImagesWindow.h"
#include "ModelAcquisitionWindow.h"
#include "ModelAcquisitionController.h"

#include <ntk/ntk.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/mesh_viewer.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/detection/plane_estimator.h>
#include <ntk/mesh/table_object_rgbd_modeler.h>

#include <QMainWindow>
#include <QImage>
#include <QApplication>

#include <fstream>

// FIXME: disabled because of flann name conflict with PCL.
// using namespace cv;
using cv::Point3f;
using cv::Vec3f;
using cv::Vec3b;
using namespace ntk;
using namespace pcl;

GuiController :: GuiController(ntk::RGBDGrabber& producer,
                               ntk::RGBDProcessor& processor)
    : m_grabber(producer),
      m_processor(processor),
      m_frame_recorder(0),
      m_mesh_generator(0),
      m_plane_estimator(0),
      m_model_acquisition_controller(0),
      m_last_tick(cv::getTickCount()),
      m_frame_counter(0),
      m_frame_rate(0),
      m_modeling_window_grabber("/tmp/demo3d/modeling"),
      m_view3d_window_grabber("/tmp/demo3d/view3d"),
      m_raw_window_grabber("/tmp/demo3d/raw"),
      m_screen_capture_mode(false),
      m_grab_frames(false),
      m_paused(false),
      m_process_one_frame(false),
      m_show_plane(false),
      m_new_model(false)
{
    m_raw_images_window = new RawImagesWindow(*this);
    m_view3d_window = new View3DWindow(*this);
    m_filters_window = new FiltersWindow(*this);
    m_model_window = new ModelAcquisitionWindow(*this);

    m_raw_images_window->show();
    m_model_window->show();
}

GuiController :: ~GuiController()
{
    delete m_raw_images_window;
    delete m_view3d_window;
    delete m_filters_window;
    delete m_model_window;
}

void GuiController :: quit()
{
    m_grabber.stop();
    m_grabber.disconnectFromDevice();
    QApplication::quit();
}

void GuiController :: setModelAcquisitionController(ModelAcquisitionController& controller)
{
    m_model_acquisition_controller = &controller;
    m_raw_images_window->ui->action_Show_Modeler->setEnabled(true);
}

void GuiController :: setFrameRecorder(ntk::RGBDFrameRecorder& frame_recorder)
{
    m_frame_recorder = &frame_recorder;
    m_raw_images_window->ui->outputDirText->setText(m_frame_recorder->directory().path());
}

void GuiController :: setMeshGenerator(MeshGenerator& generator)
{
    m_mesh_generator = &generator;
    m_raw_images_window->ui->action_3D_View->setEnabled(true);
}

static QImage toNormalizedQImage(const cv::Mat1f& m)
{
    cv::Mat1b norm;
    cv::normalize(m, norm, 0, 255, cv::NORM_MINMAX, 0);

    QImage qim (norm.cols, norm.rows, QImage::Format_RGB32);
    for (int r = 0; r < norm.rows; ++r)
        for (int c = 0; c < norm.cols; ++c)
        {
            int v = norm(r,c);
            qim.setPixel(c,r, qRgb(v,v,v));
        }
    return qim;
}

static QImage toQImage(const cv::Mat3b& m)
{
    QImage qim (m.cols, m.rows, QImage::Format_RGB32);
    for (int r = 0; r < m.rows; ++r)
    {
        QRgb* ptr = (QRgb*) qim.scanLine(r);
        for (int c = 0; c < m.cols; ++c)
        {
            const Vec3b& v = m(r,c);
            *ptr = qRgb(v[2],v[1],v[0]);
            ++ptr;
        }
    }
    return qim;
}

void GuiController :: saveCurrentFrame()
{
    std::string frame_dir = m_frame_recorder->getNextFrameDirectory();
    m_frame_recorder->saveCurrentFrame(lastImage());
}

void GuiController :: newModelCallback()
{
    cv::Mat3b obj_view;
    m_last_image.rgb().copyTo(obj_view);
    cv::RNG rng;
    std::list<cv::Rect> rects;
    std::vector<ImageWidget::TextData> texts;

    for (int i = 0; i < m_objects.size(); ++i)
    {
        Vec3b color (rng(255), rng(255), rng(255));
        modelAcquisitionWindow()->ui->mesh_view->addMesh(m_objects[i].mesh, Pose3D(), MeshViewer::FLAT);

        foreach_idx(k, m_objects[i].pixels)
        {
            int y = m_objects[i].pixels[k].y;
            int x = m_objects[i].pixels[k].x;
            if (is_yx_in_range(obj_view, y, x))
                obj_view(y, x) = color;
        }

        rects.push_back(m_objects[i].bbox);
        ImageWidget::TextData& text = m_objects[i].text;
        text.color = Vec3b(255,255,255);
        texts.push_back(text);
    }
    modelAcquisitionWindow()->ui->mesh_view->swapScene();
    modelAcquisitionWindow()->ui->object_view->setRects(rects, cv::Vec3b(0,0,255));
    modelAcquisitionWindow()->ui->object_view->setTexts(texts);
    modelAcquisitionWindow()->ui->object_view->setImage(obj_view);
}

void GuiController :: onRGBDDataUpdated()
{
    if (m_paused && !m_process_one_frame)
        return;

    m_process_one_frame = false;

    ++m_frame_counter;
    if (m_frame_counter == 10)
    {
        double current_tick = cv::getTickCount();
        m_frame_rate = m_frame_counter / ((current_tick - m_last_tick)/cv::getTickFrequency());
        m_last_tick = current_tick;
        m_frame_counter = 0;
    }

    m_grabber.copyImageTo(m_last_image);
    TimeCount tc_rgbd_process("m_processor.processImage", 2);
    m_processor.processImage(m_last_image);
    tc_rgbd_process.stop();

    if (m_raw_images_window->isVisible())
        m_raw_images_window->update(m_last_image);

    if (m_frame_recorder && (m_screen_capture_mode || m_grab_frames))
    {
        saveCurrentFrame();
    }

    if (m_screen_capture_mode)
        m_raw_window_grabber.saveFrame(QPixmap::grabWindow(m_raw_images_window->winId()));

    if (m_plane_estimator && m_show_plane)
    {
        cv::Mat1b m_plane_image (m_last_image.amplitude().cols,m_last_image.amplitude().rows);
        m_plane_estimator->estimate(m_last_image, m_plane_image);
        ntk::Mesh plane_mesh;
        generate_mesh_from_plane(plane_mesh, m_plane_estimator->currentPlane(), Point3f(0,0,-1), 1);
        m_view3d_window->ui->mesh_view->addMesh(plane_mesh, Pose3D(), MeshViewer::FLAT);
    }

    if (m_mesh_generator && m_view3d_window->isVisible())
    {
        TimeCount tc_generate ("Mesh generate");
        m_mesh_generator->generate(m_last_image);
        tc_generate.stop();

        TimeCount tc_add_mesh ("Add mesh");
        m_view3d_window->ui->mesh_view->addMesh(m_mesh_generator->mesh(), Pose3D(), MeshViewer::FLAT);
        tc_add_mesh.stop();

        if (m_screen_capture_mode)
            m_view3d_window_grabber.saveFrame(QPixmap::grabWindow(m_view3d_window->winId()));
        m_view3d_window->ui->mesh_view->swapScene();
    }

    if (m_model_acquisition_controller && m_model_window->isVisible())
    {
        bool new_model = m_model_acquisition_controller->newFrame(m_last_image);
        if (m_screen_capture_mode)
            m_modeling_window_grabber.saveFrame(QPixmap::grabWindow(m_model_window->winId()));
    }

    if (m_new_model)
    {
        newModelCallback();
        m_new_model = false;
    }

    QString status = QString("Final fps = %1 fps GRABBER = %2")
            .arg(m_frame_rate, 0, 'f', 1)
            .arg(m_grabber.frameRate(), 0, 'f', 1);
    if (m_grab_frames)
        status += " [GRABBING]";
    m_raw_images_window->ui->statusbar->showMessage(status);

    if (!m_paused)
        m_grabber.newEvent();
}

void GuiController :: on_depth_mouse_moved(int x, int y)
{
    if (!m_last_image.depth().data || x < 0 || y < 0)
        return;
    QString s = QString("Distance at (%1,%2) = %3 m")
            .arg(x).arg(y).arg(m_last_image.depth()(y,x), 0, 'f', 3);
    m_raw_images_window->ui->distanceLabel->setText(s);
}

void GuiController::toggleView3d(bool active)
{
    if (active)
    {
        m_view3d_window->show();
    }
    else
    {
        m_view3d_window->hide();
    }
}

void GuiController::toggleFilters(bool active)
{
    if (active)
    {
        m_filters_window->show();
    }
    else
    {
        m_filters_window->hide();
    }
}

void GuiController::toggleModeler(bool active)
{
    if (active)
    {
        m_model_window->show();
    }
    else
    {
        m_model_window->hide();
    }
}

void GuiController::acquireNewModels()
{
    TableObjectDetector<PointXYZ> detector;
    detector.setDepthLimits(-2, -0.5);
    detector.setObjectVoxelSize(0.003); // 3 mm voxels.
    detector.setObjectHeightLimits(0.02, 0.5);

    PointCloud<PointXYZ> cloud;
    rgbdImageToPointCloud(cloud, m_last_image);
    bool ok = detector.detect(cloud);
    if (!ok)
    {
        ntk_dbg(0) << "No clusters on a table plane found.";
        return;
    }

    m_objects.clear();
    for (int cluster_id = 0; cluster_id < detector.objectClusters().size(); ++cluster_id)
    {
        ObjectData data;
        TableObjectRGBDModeler modeler;
        modeler.feedFromTableObjectDetector(detector, cluster_id);
        Pose3D pose = *m_last_image.calibration()->depth_pose;
        modeler.addNewView(m_last_image, pose);
        modeler.computeMesh();
        float volume = modeler.meshVolume() * 100*100*100; // cm3
        modeler.computeSurfaceMesh();
        modeler.computeAccurateVerticeColors();
        cv::Rect bbox;
        foreach_idx(i, modeler.currentMesh().vertices)
        {
            cv::Point3f p = m_last_image.calibration()->rgb_pose->projectToImage(modeler.currentMesh().vertices[i]);
            if (bbox.area() < 1)
                bbox = cv::Rect(p.x, p.y, 1, 1);
            else
                bbox |= cv::Rect(p.x, p.y, 1, 1);
            data.pixels.push_back(cv::Point2i(ntk::math::rnd(p.x), ntk::math::rnd(p.y)));
        }
        data.bbox = bbox;
        data.text.text = cv::format("Volume = %d cm3", ntk::math::rnd(volume));
        data.text.x = bbox.x;
        data.text.y = bbox.y;
        data.mesh = modeler.currentMesh();
        m_objects.push_back(data);
    }
    notifyNewModel();
}

void GuiController :: resetModels()
{
    m_objects.clear();
    modelAcquisitionWindow()->ui->mesh_view->swapScene();
}

void GuiController :: saveModels()
{
    for (int i = 0; i < m_objects.size(); ++i)
        m_objects[i].mesh.saveToPlyFile(cv::format("object%d.ply", i).c_str());
}
