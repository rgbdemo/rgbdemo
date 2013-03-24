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

#ifndef PEOPLETRACKERWINDOW_H
#define PEOPLETRACKERWINDOW_H

#include <ntk/mesh/mesh_generator.h>
#include <ntk/gui/image_widget.h>

#include "PeopleTracker.h"

#include <QMainWindow>

namespace Ui {
    class PeopleTrackerWindow;
}

class GuiController;

class PeopleTrackerImageWidget : public ntk::ImageWidget
{
  Q_OBJECT

public:
  struct LocatedText
  {
    cv::Point pos;
    std::string text;
  };

  PeopleTrackerImageWidget(QWidget* parent) : ntk::ImageWidget(parent)
  {}

  void updateTextFromDetections(const std::vector<PeopleTracker::PersonDetection>& detections,
                                const ntk::Pose3D& depth_pose);
  virtual void paintEvent(QPaintEvent * event);

private:
  std::vector<LocatedText> m_current_texts;
};

class PeopleTrackerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit PeopleTrackerWindow(GuiController& controller,
                                 PeopleTracker& tracker,
                                 QWidget *parent = 0);
    ~PeopleTrackerWindow();

public:
  void update(const ntk::RGBDImage& image);

private slots:
  void on_setAsBackgroundButton_clicked();
  void on_saveMeshPushButton_clicked();
  void on_trianglePushButton_clicked();
  void on_pointCloudPushButton_clicked();
  void on_showPlaneCheckBox_toggled(bool checked);
  void on_colorMappingCheckBox_toggled(bool checked);
  void on_resetCamera_clicked();

  void on_loadParametersButton_clicked();

  void on_actionNext_frame_triggered();

private:
    Ui::PeopleTrackerWindow *ui;
    GuiController& m_controller;
    ntk::MeshGenerator m_mesh_generator;
    bool m_show_plane;
    PeopleTracker& m_tracker;
};

#endif // PEOPLETRACKERWINDOW_H
