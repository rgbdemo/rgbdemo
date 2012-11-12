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

#include "ModelAcquisitionController.h"
#include "ModelAcquisitionWindow.h"
#include "ui_ModelAcquisitionWindow.h"

#include "GuiController.h"

#include <ntk/utils/time.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/mesh/mesh_viewer.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/plane_remover.h>

#include <QtConcurrentRun>

using namespace ntk;
using namespace cv;

void ModelAcquisitionController :: grabAndMove()
{
    m_controller.frameRecorder()->saveCurrentFrame(m_controller.lastImage());
    move();
}

void ModelAcquisitionController :: modelAndMove()
{
    newFrame(m_controller.lastImage());
    move();
}

void ModelAcquisitionController :: reset()
{
    m_modeler.reset();
    m_pose_estimator->reset();
    m_controller.modelAcquisitionWindow()->ui->mesh_view->swapScene();
}

// WARNING: image must be a pointer because QTConcurrent::run gives argument
// by copy!!
bool ModelAcquisitionController :: newFrameThread(const ntk::RGBDImage* image)
{
    m_pose_estimator->addNewImage(*image);
    bool pose_ok = m_pose_estimator->estimateCurrentPose();
    if (!pose_ok)
        return false;
#if 0
    MeshGenerator generator;
    generator.setUseColor(true);
    Pose3D rgb_pose = m_pose_estimator->currentPose();
    rgb_pose.toRightCamera(image->calibration()->rgb_intrinsics, image->calibration()->R, image->calibration()->T);
    generator.setMeshType(MeshGenerator::PointCloudMesh);
    generator.setMaxDeltaDepthBetweenEdges(0.01);
    generator.generate(*image,
                       m_pose_estimator->currentPose(),
                       rgb_pose);
    static int frame_id = 0;
    // generator.mesh().saveToPlyFile(cv::format("/tmp/debug_%04d.ply", frame_id).c_str());
    ++frame_id;
#endif

    m_modeler.addNewView(*image, m_pose_estimator->currentPose());
    m_modeler.computeMesh();
    m_current_mesh = m_modeler.currentMesh();
    return true;
}

void ModelAcquisitionController :: newFrame(const ntk::RGBDImage& image)
{
    if (m_paused)
        return;

    if (!m_pose_estimator)
    {
        ntk_dbg(1) << "Warning: no pose estimator";
        return;
    }

    if (!m_new_frame_run.isFinished())
    {
        return;
    }
    else
    {        
        image.copyTo(m_current_image);
        m_controller.rgbdProcessor().computeNormalsPCL(m_current_image);
        m_new_frame_run = QtConcurrent::run(this, &ModelAcquisitionController::newFrameThread, &m_current_image);
        m_new_frame_run.waitForFinished();
        if (m_new_frame_run.isStarted() && m_new_frame_run.resultCount() > 0)
        {
            if (m_new_frame_run.result())
            {
                cv::Mat3b im_with_markers;
                m_current_image.rgb().copyTo(im_with_markers);
                m_current_image.setEstimatedWorldDepthPose(m_pose_estimator->currentPose());
                m_pose_estimator->drawDetectedMarkers(im_with_markers);
                m_controller.modelAcquisitionWindow()->ui->marker_image->setImage(im_with_markers);
                m_controller.modelAcquisitionWindow()->ui->mesh_view->addMesh(m_current_mesh, Pose3D(), MeshViewer::FLAT);
                m_controller.modelAcquisitionWindow()->ui->mesh_view->swapScene();
            }
        }
        // m_controller.modelAcquisitionController()->setPaused(true);

    }
}

void ModelAcquisitionController :: removeFloorPlane()
{
    PlaneRemover remover;
    remover.setInputPlane(m_floor_plane);
    remover.removePlane(m_current_mesh);
    m_controller.modelAcquisitionWindow()->ui->mesh_view->addMesh(m_current_mesh, Pose3D(), MeshViewer::FLAT);
    m_controller.modelAcquisitionWindow()->ui->mesh_view->swapScene();
}
