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

#include "ModelAcquisitionWindow.h"
#include "ui_ModelAcquisitionWindow.h"

#include "GuiController.h"
#include "ModelAcquisitionController.h"

#include <ntk/utils/time.h>

ModelAcquisitionWindow::ModelAcquisitionWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ModelAcquisitionWindow),
    m_controller(controller)
{
    ui->setupUi(this);
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

void ModelAcquisitionWindow::on_resetModelsButton_clicked()
{
    m_controller.resetModels();
}

void ModelAcquisitionWindow::on_saveMeshButton_clicked()
{
    m_controller.saveModels();
}

void ModelAcquisitionWindow::on_acquireModelsButton_clicked()
{
    m_controller.acquireNewModels();
}
