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

#ifndef VIEW3DWINDOW_H
#define VIEW3DWINDOW_H

#include <QMainWindow>

namespace Ui {
    class View3DWindow;
}

class GuiController;

class View3DWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit View3DWindow(GuiController& controller, QWidget *parent = 0);
    ~View3DWindow();

private:
    Ui::View3DWindow *ui;
    GuiController& m_controller;

private slots:
    void on_saveMeshPushButton_clicked();
    void on_trianglePushButton_clicked();
    void on_surfelsPushButton_clicked();
    void on_pointCloudPushButton_clicked();
    void on_colorMappingCheckBox_toggled(bool checked);
    void on_resetCamera_clicked();

    friend class GuiController;
};

#endif // VIEW3DWINDOW_H
