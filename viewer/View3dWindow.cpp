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

#include "View3dWindow.h"
#include "ui_View3dWindow.h"

#include "GuiController.h"

#include <ntk/mesh/mesh_generator.h>

View3DWindow::View3DWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),    
    ui(new Ui::View3DWindow),
    m_controller(controller)
{  
  ui->setupUi(this);
}

View3DWindow::~View3DWindow()
{
  delete ui;
}

void View3DWindow::on_resetCamera_clicked()
{
  ui->mesh_view->resetCamera();
  ui->mesh_view->updateGL();
}

void View3DWindow::on_colorMappingCheckBox_toggled(bool checked)
{
  m_controller.meshGenerator()->setUseColor(checked);
}

void View3DWindow::on_pointCloudPushButton_clicked()
{
  m_controller.meshGenerator()->setMeshType(ntk::MeshGenerator::PointCloudMesh);
}

void View3DWindow::on_surfelsPushButton_clicked()
{
  m_controller.meshGenerator()->setMeshType(ntk::MeshGenerator::SurfelsMesh);
}

void View3DWindow::on_trianglePushButton_clicked()
{
  m_controller.meshGenerator()->setMeshType(ntk::MeshGenerator::TriangleMesh);
}

void View3DWindow::on_saveMeshPushButton_clicked()
{
  m_controller.meshGenerator()->mesh().saveToPlyFile("current_mesh.ply");
}
