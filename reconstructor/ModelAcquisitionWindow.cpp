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

#include "ModelAcquisitionWindow.h"
#include "ui_ModelAcquisitionWindow.h"

#include "GuiController.h"
#include "ModelAcquisitionController.h"

#include <ntk/utils/time.h>

ModelAcquisitionWindow::ModelAcquisitionWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ModelAcquisitionWindow),
    m_controller(controller),
    m_angle_delta(10),
    m_iteration(0)
{
  ui->setupUi(this);
  // ui->mesh_view->enableLighting();
}

ModelAcquisitionWindow::~ModelAcquisitionWindow()
{
  delete ui;
}

void ModelAcquisitionWindow::on_resetCamera_clicked()
{
  ui->mesh_view->resetCamera();
  ui->mesh_view->updateGL();
}

void ModelAcquisitionWindow::on_action_Grab_and_Move_triggered()
{
  m_controller.modelAcquisitionController()->grabAndMove();
}

void ModelAcquisitionWindow::on_actionModel_and_Move_triggered()
{
  m_controller.modelAcquisitionController()->modelAndMove();
}

void ModelAcquisitionWindow::on_action_Grab_Sequence_triggered()
{
  if (m_iteration == 0)
  {
    m_controller.modelAcquisitionController()->reset();
  }

  m_controller.modelAcquisitionController()->grabAndMove();

  if (m_iteration < m_controller.modelAcquisitionController()->getNumberOfSteps())
  {
    ++m_iteration;
    ui->action_Grab_Sequence->trigger();
  }
  else
    m_iteration = 0;
}

void ModelAcquisitionWindow::on_stopSequenceButton_clicked()
{
  m_iteration = INT_MAX;
}

void ModelAcquisitionWindow::on_resetModelButton_clicked()
{
  m_iteration = 0;
  m_controller.modelAcquisitionController()->reset();
}

void ModelAcquisitionWindow::on_resolutionSpinBox_editingFinished()
{
  m_controller.modelAcquisitionController()->modeler().setResolution(ui->resolutionSpinBox->value());
}

void ModelAcquisitionWindow::on_depthMarginSpinBox_editingFinished()
{
  m_controller.modelAcquisitionController()->modeler().setDepthMargin(ui->depthMarginSpinBox->value());
}

void ModelAcquisitionWindow::on_actionModel_Sequence_triggered()
{
  if (m_iteration == 0)
  {
    m_controller.modelAcquisitionController()->reset();
  }

  m_controller.modelAcquisitionController()->modelAndMove();

  if (m_iteration < m_controller.modelAcquisitionController()->getNumberOfSteps())
  {
    ++m_iteration;
    ui->actionModel_Sequence->trigger();
  }
  else
    m_iteration = 0;
}

void ModelAcquisitionWindow::on_saveMeshButton_clicked()
{
  m_controller.modelAcquisitionController()->modeler().computeSurfaceMesh();
  m_controller.modelAcquisitionController()->modeler()
      .currentMesh().saveToPlyFile("scene_mesh.ply");
}

void ModelAcquisitionWindow::on_startButton_clicked()
{
  m_controller.modelAcquisitionController()->setPaused(false);
}

void ModelAcquisitionWindow::on_stopButton_clicked()
{
  m_controller.modelAcquisitionController()->setPaused(true);
}

void ModelAcquisitionWindow::on_resetButton_clicked()
{
  m_controller.modelAcquisitionController()->reset();
}
