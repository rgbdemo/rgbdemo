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

#include "ModelAcquisitionWindow.h"
#include "ui_ModelAcquisitionWindow.h"

#include "GuiController.h"
#include "ModelAcquisitionController.h"

#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/utils/time.h>

#include <QFileDialog>

#include <fstream>

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

void ModelAcquisitionWindow::on_saveMeshButton_clicked()
{
    if (!m_controller.modelAcquisitionController()->currentImage().withRawDepthDataAndCalibrated())
    {
        ntk_dbg(1) << "No image already processed.";
        return;
    }


    QString filename = QFileDialog::getSaveFileName(this,
                                                    "Save model as...",
                                                    QString("object1.model"));
    QDir dir;
    dir.mkpath(filename);
    std::string model_path = filename.toStdString();
    ntk::RGBDFrameRecorder recorder (model_path);
    recorder.setSaveRgbPose(true);
    recorder.saveCurrentFrame(m_controller.modelAcquisitionController()->currentImage());
    m_controller.modelAcquisitionController()->currentMesh().saveToPlyFile((model_path + "/mesh.ply").c_str());
    m_controller.modelAcquisitionController()->currentImage().calibration()->saveToFile((model_path + "/calibration.yml").c_str());
}

void ModelAcquisitionWindow::on_addCurrentFrameButton_clicked()
{
    m_controller.modelAcquisitionController()->setPaused(false);
}

void ModelAcquisitionWindow::on_resetButton_clicked()
{
    m_controller.modelAcquisitionController()->reset();
}

void ModelAcquisitionWindow::on_removeFloorPlaneButton_clicked()
{
    m_controller.modelAcquisitionController()->removeFloorPlane();
}
