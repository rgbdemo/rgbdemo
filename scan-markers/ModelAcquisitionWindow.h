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

#ifndef MODELACQUISITIONWINDOW_H
#define MODELACQUISITIONWINDOW_H

#include <QMainWindow>

namespace Ui {
    class ModelAcquisitionWindow;
}

class GuiController;

class ModelAcquisitionWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ModelAcquisitionWindow(GuiController& controller, QWidget *parent = 0);
    ~ModelAcquisitionWindow();

private:
    Ui::ModelAcquisitionWindow *ui;
    GuiController& m_controller;

private slots:
    void on_saveMeshButton_clicked();
    void on_resetCamera_clicked();

    void on_addCurrentFrameButton_clicked();
    void on_resetButton_clicked();

    void on_removeFloorPlaneButton_clicked();

private:
    float m_angle_delta;
    int m_iteration;

    friend class ModelAcquisitionController;
    friend class FrameModelAcquisitionController;
};

#endif // MODELACQUISITIONWINDOW_H
