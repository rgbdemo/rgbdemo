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

#include "ModelAcquisitionController.h"
#include "ModelAcquisitionWindow.h"
#include "ui_ModelAcquisitionWindow.h"

#include "GuiController.h"

#include <ntk/utils/time.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/mesh/mesh_viewer.h>

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

bool ModelAcquisitionController :: newFrameThread(const ntk::RGBDImage& image)
{
  bool pose_ok = m_pose_estimator->estimateNewPose(image);
  if (!pose_ok)
    return false;
  m_modeler.addNewView(image, m_pose_estimator->currentPose());
  m_modeler.computeMesh();
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
    if (m_new_frame_run.isStarted() && m_new_frame_run.resultCount() > 0)
    {
      if (m_new_frame_run.result())
      {
        m_controller.modelAcquisitionWindow()->ui->mesh_view->addMesh(m_modeler.currentMesh(), Pose3D(), MeshViewer::FLAT);
        m_controller.modelAcquisitionWindow()->ui->mesh_view->swapScene();
      }
    }
    image.copyTo(m_current_image);
    m_new_frame_run = QtConcurrent::run(this, &ModelAcquisitionController::newFrameThread, m_current_image);
    m_new_frame_run.waitForFinished();
  }
}

void Pa10ModelAcquisitionController :: move()
{
  ntk::Pa10client::s1s2s3e1e2w1w2_t angles = m_controller.pa10Angles();
  angles.w2 += m_delta_angle;
  m_controller.setPa10Angles(angles);
  ntk::sleep(100);
}

void Pa10ModelAcquisitionController :: reset()
{
  ModelAcquisitionController::reset();
  ntk::Pa10client::s1s2s3e1e2w1w2_t angles = m_controller.pa10Angles();
  angles.w2 = 0;
  m_controller.setPa10Angles(angles);
}

void Pa10PoseAcquisitionController :: move()
{
  ntk::Pa10client::s1s2s3e1e2w1w2_t angles = m_controller.pa10Angles();
  angles.w2 += 0;
  angles.e1 += 3;
  angles.s1 += 3;
  m_controller.setPa10Angles(angles);
  ntk::sleep(100);
}

void Pa10PoseAcquisitionController :: reset()
{
  ModelAcquisitionController::reset();
  ntk::Pa10client::s1s2s3e1e2w1w2_t angles = m_controller.pa10Angles();
  angles.w2 = -10;
  angles.e1 = 90;
  angles.s1 = 100;
  m_controller.setPa10Angles(angles);
}
