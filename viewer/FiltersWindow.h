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

#ifndef FILTERSWINDOW_H
#define FILTERSWINDOW_H

#include <QMainWindow>

namespace Ui {
    class FiltersWindow;
}

class GuiController;

class FiltersWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit FiltersWindow(GuiController& controller, QWidget *parent = 0);
    ~FiltersWindow();

  private:
    void updateDepthSlider();

private:
    Ui::FiltersWindow *ui;
    GuiController& m_controller;

private slots:
    void on_removeSmallStructuresBox_toggled(bool checked);
    void on_fillSmallHolesCheckBox_toggled(bool checked);
    void on_maxDepthSlider_valueChanged(int value);
    void on_minDepthSlider_valueChanged(int value);
    void on_unstableCheckBox_toggled(bool checked);
    void on_normalsCheckBox_toggled(bool checked);
    void on_medianCheckBox_toggled(bool checked);
    void on_edgesCheckBox_toggled(bool checked);
    void on_depthThresholdCheckBox_toggled(bool checked);
    void on_kinectTiltSlider_valueChanged(int value);
};

#endif // FILTERSWINDOW_H
