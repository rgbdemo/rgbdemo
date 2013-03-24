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

#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <ntk/camera/rgbd_image.h>
#include <ntk/camera/calibration.h>

#include <QMainWindow>

class GuiController;

namespace Ui {
class CalibrationWindow;
}

class CalibrationWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit CalibrationWindow(GuiController& controller, QWidget *parent = 0);
    ~CalibrationWindow();
    
private slots:
    void on_addFrameButton_clicked();
    void on_calibrateButton_clicked();
    void on_resetButton_clicked();
    void on_saveCalibrationButton_clicked();

private:
    Ui::CalibrationWindow *ui;
    GuiController& controller;
    friend class GuiController;
    std::vector<ntk::RGBDImage> images;
    ntk::RGBDCalibration last_calibration;
};

#endif // CALIBRATIONWINDOW_H
