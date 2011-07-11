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

#include "View3dWindow.h"
#include "ui_View3dWindow.h"

#include "GuiController.h"

#include <ntk/mesh/mesh_generator.h>

using namespace cv;
using namespace ntk;

View3DWindow::View3DWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),    
    ui(new Ui::View3DWindow),
    m_controller(controller)
{  
    ui->setupUi(this);
    ui->mesh_view->window = this;
}

View3DWindow::~View3DWindow()
{
    delete ui;
}

void View3DWindow :: getCalibration(cv::Vec3f& t, cv::Vec3f& r) const
{
    t[0] = ui->txValue->value();
    t[1] = ui->tyValue->value();
    t[2] = ui->tzValue->value();
    r[0] = deg_to_rad(ui->rxValue->value());
    r[1] = deg_to_rad(ui->ryValue->value());
    r[2] = deg_to_rad(ui->rzValue->value());
}

void View3DWindow :: updateFromCalibration(const cv::Vec3f& t, const cv::Vec3f& r)
{    
    ui->txValue->setValue(t[0]);
    ui->tyValue->setValue(t[1]);
    ui->tzValue->setValue(t[2]);
    ui->rxValue->setValue(rad_to_deg(r[0]));
    ui->ryValue->setValue(rad_to_deg(r[1]));
    ui->rzValue->setValue(rad_to_deg(r[2]));
}

void View3DWindow :: updateToCalibration()
{
    cv::Vec3f r;
    cv::Vec3f t;
    t[0] = ui->txValue->value();
    t[1] = ui->tyValue->value();
    t[2] = ui->tzValue->value();
    r[0] = deg_to_rad(ui->rxValue->value());
    r[1] = deg_to_rad(ui->ryValue->value());
    r[2] = deg_to_rad(ui->rzValue->value());
    m_controller.updateCameraCalibration(t, r);
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

void View3DWindow::on_resolutionFactorSpinBox_valueChanged(double value)
{
    m_controller.meshGenerator()->setResolutionFactor(value);
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

void View3DWindow :: on_mergeViewsCheckBox_toggled(bool checked)
{
    m_controller.setMergeViews(checked);
}

void View3DWindow :: on_calibrationModeCheckBox_toggled(bool checked)
{
    ui->mesh_view->setCalibrationMode(checked);
}

void View3DWindow :: on_refineWithICPButton_clicked()
{
    m_controller.refineCalibrationWithICP();
}

void View3DWindow::on_txValue_editingFinished() { updateToCalibration(); }
void View3DWindow::on_tyValue_editingFinished() { updateToCalibration(); }
void View3DWindow::on_tzValue_editingFinished() { updateToCalibration(); }
void View3DWindow::on_rxValue_editingFinished() { updateToCalibration(); }
void View3DWindow::on_ryValue_editingFinished() { updateToCalibration(); }
void View3DWindow::on_rzValue_editingFinished() { updateToCalibration(); }

void CalibrationMeshViewer::onCameraPositionUpdate(const cv::Vec3f &translation, const cv::Vec3f &rotation)
{
    if (!m_calibration_mode)
    {
        MeshViewer::onCameraPositionUpdate(translation, rotation);
        return;
    }

    GLdouble m[16];
    GLdouble deltam[16];

    const float rotation_scale = 0.2;
    const float translation_scale = 0.2;

    // Get the delta transformation is visualization frame.
    makeCurrent();
    glMatrixMode(GL_MODELVIEW);
    glGetDoublev(GL_MODELVIEW_MATRIX, m);
    glLoadIdentity();
    glTranslatef(translation_scale*translation[0],translation_scale*translation[1],translation_scale*translation[2]);
    glTranslatef(m_display_center.x,m_display_center.y,m_display_center.z);
    glRotatef(rotation_scale*rotation[0], 0,1,0);
    glRotatef(rotation_scale*rotation[1], 1,0,0);
    glTranslatef(-m_display_center.x,-m_display_center.y,-m_display_center.z);
    glGetDoublev(GL_MODELVIEW_MATRIX, deltam);
    glLoadMatrixd(m);

    cv::Vec3f t,r;
    window->getCalibration(t, r);
    Pose3D p_old;
    p_old.applyTransformBefore(t, r);

    cv::Mat1d H_old = p_old.cvCameraTransformd();
    cv::Mat1d H(4,4,(double*)deltam); H = H.t(); // delta rotation AFTER model view matrix
    cv::Mat1d M(4,4,(double*)m); M = M.t(); // model view matrix

    cv::Mat1d Hp = (M.inv() * H * M * H_old.inv()).inv(); // delta rotation BEFORE model view matrix

    Pose3D p;
    p.setCameraTransform(Hp);

    window->updateFromCalibration(p.cvTranslation(), p.cvEulerRotation());
    window->updateToCalibration();
}
