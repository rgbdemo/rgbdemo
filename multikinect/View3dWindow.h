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

#ifndef VIEW3DWINDOW_H
#define VIEW3DWINDOW_H

#include <ntk/camera/calibration.h>
#include <ntk/mesh/mesh_viewer.h>

#include <QMainWindow>

namespace Ui {
    class View3DWindow;
}

class GuiController;

class View3DWindow;
class CalibrationMeshViewer : public ntk::MeshViewer
{
public:
    CalibrationMeshViewer(QWidget* parent = 0)
        : ntk::MeshViewer(parent),
          m_calibration_mode(false)
    {}

    void setCalibrationMode(bool enable) { m_calibration_mode = enable; }

protected:
    virtual void onCameraPositionUpdate(const cv::Vec3f& translation, const cv::Vec3f& rotation);

public:
    View3DWindow* window;
    bool m_calibration_mode;
};

class View3DWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit View3DWindow(GuiController& controller, QWidget *parent = 0);
    ~View3DWindow();

public:
    void updateFromCalibration(const cv::Vec3f& t, const cv::Vec3f& r);
    void updateToCalibration();
    void getCalibration(cv::Vec3f& t, cv::Vec3f& r) const;

private:
    Ui::View3DWindow *ui;
    GuiController& m_controller;

private slots:
    void on_saveMeshPushButton_clicked();
    void on_trianglePushButton_clicked();
    void on_surfelsPushButton_clicked();
    void on_pointCloudPushButton_clicked();
    void on_resolutionFactorSpinBox_valueChanged(double );
    void on_colorMappingCheckBox_toggled(bool checked);
    void on_resetCamera_clicked();
    void on_txValue_editingFinished();
    void on_tyValue_editingFinished();
    void on_tzValue_editingFinished();
    void on_rxValue_editingFinished();
    void on_ryValue_editingFinished();
    void on_rzValue_editingFinished();
    void on_mergeViewsCheckBox_toggled(bool checked);
    void on_calibrationModeCheckBox_toggled(bool checked);
    void on_refineWithICPButton_clicked();

    friend class GuiController;
    friend class CalibrationMeshViewer;
};

#endif // VIEW3DWINDOW_H
