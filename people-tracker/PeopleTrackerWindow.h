/*************************************************************************
 *
 * CONFIDENTIAL
 * __________________
 *
 * [2011] Nicolas Burrus
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Nicolas Burrus.
 *
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Nicolas Burrus <nicolas@burrus.name>.
 */

#ifndef PEOPLETRACKERWINDOW_H
#define PEOPLETRACKERWINDOW_H

#include <ntk/mesh/mesh_generator.h>

#include "PeopleTracker.h"

#include <QMainWindow>

namespace Ui {
    class PeopleTrackerWindow;
}

class GuiController;

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

private:
    Ui::PeopleTrackerWindow *ui;
    GuiController& m_controller;
    ntk::MeshGenerator m_mesh_generator;
    bool m_show_plane;
    PeopleTracker& m_tracker;
};

#endif // PEOPLETRACKERWINDOW_H
