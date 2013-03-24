/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
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
  newFrameImpl(m_controller.lastImage());
  move();
}

void ModelAcquisitionController :: reset()
{
    m_modeler.reset();
    m_pose_estimator->reset();
    m_controller.modelAcquisitionWindow()->ui->mesh_view->swapScene();
}

bool ModelAcquisitionController :: newFrame(const ntk::RGBDImage& image)
{
    if (m_paused)
      return false;
    return newFrameImpl(image);
}

bool ModelAcquisitionController :: newFrameImpl(const ntk::RGBDImage& image)
{
    m_pose_estimator->addNewImage(image);
    bool pose_ok = m_pose_estimator->estimateCurrentPose();
    if (!pose_ok)
        return false;
    m_modeler.addNewView(image, m_pose_estimator->currentPose());
    m_modeler.computeMesh();
    m_controller.notifyNewModel();
    return true;
}
