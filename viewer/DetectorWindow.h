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

#ifndef DETECTORWINDOW_H
#define DETECTORWINDOW_H

#include <ntk/camera/calibration.h>
#include <QMainWindow>

namespace Ui {
    class DetectorWindow;
}

class GuiController;

class DetectorWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit DetectorWindow(GuiController& controller, QWidget *parent = 0);
    ~DetectorWindow();

    void update(const ntk::RGBDImage& image);
    void setThresholdValues(int min_value, int max_value);

private:
    void updateThreshold();

private:
    Ui::DetectorWindow *ui;
    GuiController& m_controller;

private slots:
    void on_maxThresholdSlider_valueChanged(int value);
    void on_minThresholdSlider_valueChanged(int value);
};

#endif // DETECTORWINDOW_H
