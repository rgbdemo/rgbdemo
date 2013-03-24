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

#ifndef OBJECT_OBJECT_WINDOW_H
#define OBJECT_OBJECT_WINDOW_H

#include <ntk/core.h>

#include <QMainWindow>

#include <ntk/geometry/relative_pose_estimator.h>
#include <ntk/detection/object/object_finder.h>

namespace Ui {
    class ObjectWindow;
}

class GuiController;

class ObjectWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ObjectWindow(GuiController& controller, QWidget *parent = 0);
    ~ObjectWindow();

public:
    void setDatabasePath(const std::string& path);
    void registerNewModel(const ntk::RGBDImage& image);
    void processNewFrame(const ntk::RGBDImage& image);
    void updateView();

private slots:
    void on_resetCamera_clicked();
    void on_detectButton_clicked();

    void on_databasePathLineEdit_editingFinished();

    void on_browseButton_clicked();

    void on_showObjectCheckBox_toggled(bool checked);

private:
    void initializeDetector();
    void resetDetector();

private:
    Ui::ObjectWindow *ui;
    GuiController& m_controller;
    std::string m_database_path;
    ntk::ObjectFinderPtr m_object_finder;
    ntk::Mesh m_table_mesh;
    ntk::Mesh m_object_mesh;
    ntk::Mesh m_scene_mesh;
    int m_current_model_index;
    bool m_show_detection;

private:
    friend class GuiController;
    RecursiveQMutex m_lock;
};

#endif // OBJECT_OBJECT_WINDOW_H
