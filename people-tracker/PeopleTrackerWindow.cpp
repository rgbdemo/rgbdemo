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

#include "PeopleTrackerWindow.h"
#include "ui_PeopleTrackerWindow.h"

#include "GuiController.h"

#include <ntk/geometry/pose_3d.h>

#include <QFileDialog>

using namespace cv;
using namespace ntk;

PeopleTrackerWindow::PeopleTrackerWindow(GuiController& controller,
                                         PeopleTracker& tracker,
                                         QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::PeopleTrackerWindow),
  m_controller(controller),
  m_show_plane(false),
  m_tracker(tracker)
{
  ui->setupUi(this);
  on_colorMappingCheckBox_toggled(ui->colorMappingCheckBox->checkState() == Qt::Checked);
  on_showPlaneCheckBox_toggled(ui->showPlaneCheckBox->checkState() == Qt::Checked);
}

PeopleTrackerWindow::~PeopleTrackerWindow()
{
  delete ui;
}

void PeopleTrackerWindow::update(const ntk::RGBDImage& image)
{
  m_tracker.processNewImage(image);

  Pose3D depth_pose = *image.calibration()->depth_pose;
  Pose3D rgb_pose = *image.calibration()->rgb_pose;

  // m_mesh_generator.generate(m_tracker.foregroundImage(), depth_pose, rgb_pose);
  m_mesh_generator.generate(image, depth_pose, rgb_pose);
  ui->mesh_view->addMesh(m_mesh_generator.mesh(), Pose3D(), MeshViewer::FLAT);

  foreach_idx(i, m_tracker.detections())
  {
    Point3f p = m_tracker.detections()[i].highest_point;
    float person_height = 3.0 - m_tracker.detections()[i].highest_point_in_image.z;
    ntk_dbg_print(person_height, 1);
    Mesh mesh;
    mesh.addCube(p + (Point3f)(m_tracker.groundPlane().normal()*0.1f), Point3f(0.1, 0.1, 0.1));
    mesh.colors.resize(mesh.vertices.size(), Vec3b(255,0,0));
    ui->mesh_view->addMesh(mesh, Pose3D(), MeshViewer::FLAT);
  }

  if (m_show_plane && m_tracker.isInitialized())
  {
    Mesh plane_mesh;
    plane_mesh.vertices.push_back(m_tracker.workingZone()[0]);
    plane_mesh.vertices.push_back(m_tracker.workingZone()[1]);
    plane_mesh.vertices.push_back(m_tracker.workingZone()[2]);
    plane_mesh.vertices.push_back(m_tracker.workingZone()[3]);
    plane_mesh.faces.push_back(Face(0, 1, 2));
    plane_mesh.faces.push_back(Face(0, 2, 3));

    for (int i = 0; i < 4; ++i)
      plane_mesh.addCube(m_tracker.workingZone()[i],
                         Point3f(0.2,0.2,0.2));

    plane_mesh.colors.resize(plane_mesh.vertices.size(), Vec3b(255,255,0));

    ui->mesh_view->addMesh(plane_mesh, Pose3D(), MeshViewer::FLAT);
  }
  ui->mesh_view->swapScene();
}

void PeopleTrackerWindow::on_setAsBackgroundButton_clicked()
{
  bool prev_state = m_controller.isPaused();
  m_controller.setPaused(true);
  m_tracker.estimateBackground(m_controller.lastImage());
  m_controller.setPaused(prev_state);
  QString filename = QFileDialog::getSaveFileName(this,
                                                  "Save tracker background as...",
                                                  QString("tracker_background.yml"));
  if (filename.isEmpty())
    return;
  m_tracker.saveBackgroundInfoToFile(filename.toStdString());
}

void PeopleTrackerWindow::on_resetCamera_clicked()
{
  ui->mesh_view->resetCamera();
  ui->mesh_view->updateGL();
}

void PeopleTrackerWindow::on_colorMappingCheckBox_toggled(bool checked)
{
  m_mesh_generator.setUseColor(checked);
}

void PeopleTrackerWindow::on_showPlaneCheckBox_toggled(bool checked)
{
  m_show_plane = checked;
}

void PeopleTrackerWindow::on_pointCloudPushButton_clicked()
{
  m_mesh_generator.setMeshType(ntk::MeshGenerator::PointCloudMesh);
}

void PeopleTrackerWindow::on_trianglePushButton_clicked()
{
  m_mesh_generator.setMeshType(ntk::MeshGenerator::TriangleMesh);
}

void PeopleTrackerWindow::on_saveMeshPushButton_clicked()
{
  m_mesh_generator.mesh().saveToPlyFile("current_mesh.ply");
}

void PeopleTrackerWindow::on_loadParametersButton_clicked()
{
  ntk_dbg_print(m_tracker.virtualTopView(), 1);
  QString filename = QFileDialog::getOpenFileName(this,
                                                  "Choose a tracker_background.yml file",
                                                  QString(),
                                                  QString("tracker_background.yml"));
  if (filename.isEmpty())
    return;
  m_tracker.loadBackgroundInfoFromFile(filename.toStdString());
  ntk_dbg_print(m_tracker.virtualTopView(), 1);
}
