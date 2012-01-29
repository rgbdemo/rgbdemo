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

#include "ObjectWindow.h"
#include "ui_ObjectWindow.h"

#include "GuiController.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/utils/qt_utils.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/mesh_viewer.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/mesh/pcl_utils.h>
#include <ntk/geometry/relative_pose_estimator_tabletop.h>

#include <ntk/detection/object/object_finder.h>
#include <ntk/detection/object/sift_object_match_lowe.h>

#include <QFileDialog>

using namespace ntk;
using namespace cv;

ObjectWindow::ObjectWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ObjectWindow),
    m_controller(controller),
    m_database_path("."),
    m_object_finder(false),
    m_current_model_index(0),
    m_show_detection(true)
{
  ui->setupUi(this);
}

ObjectWindow::~ObjectWindow()
{
  delete ui;
}

void ObjectWindow :: setDatabasePath(const std::string& path)
{
    ui->databasePathLineEdit->setText(path.c_str());
    resetDetector();
}

void ObjectWindow::on_resetCamera_clicked()
{
  ui->mesh_view->resetCamera();
  ui->mesh_view->updateGL();
}

void ObjectWindow :: processNewFrame(const ntk::RGBDImage& image)
{
    QMutexLocker locker(&m_lock);

    ntk::TimeCount tc_object("ObjectWindow::processNewFrame", 1);
    if (!m_object_finder)
        return;

    ntk::RGBDImage filtered_image;
    image.copyTo(filtered_image);

    for_all_rc(filtered_image.mappedDepthMask())
    {
        if (!filtered_image.mappedDepthMask()(r,c))
        {
            filtered_image.rgbAsGrayRef()(r,c) = 0;
            filtered_image.rgbRef()(r,c) = Vec3b(0,0,0);
        }
    }

    tc_object.elapsedMsecs("FILTERING: ");

    m_object_finder->processNewImage(filtered_image);

    tc_object.elapsedMsecs("FINDING OBJECTS: ");

    int num_matches = m_object_finder->objectDetector()->nbObjectMatches();
    if (num_matches < 1)
    {
        ntk_dbg(1) << "No object found";
        return;
    }

    for (int i = 0; i < num_matches; ++i)
    {
        const ObjectMatch& match = m_object_finder->objectDetector()->objectMatch(i);
        const SiftObjectMatchLowe* lowe_match = dynamic_cast<const SiftObjectMatchLowe*>(&match);
        if (lowe_match)
            ntk_dbg_print(*lowe_match, 1);
    }
    const ObjectMatch& first_match = m_object_finder->objectDetector()->objectMatch(0);
    Pose3D pose3d = first_match.pose()->pose3d();
    pose3d.toLeftCamera(image.calibration()->depth_intrinsics, image.calibration()->R, image.calibration()->T);
    ntk::Mesh mesh;
    first_match.model().loadMeshWithCache(mesh);

    tc_object.elapsedMsecs("LOADING MESH: ");

#if 0
    cv::Mat4b debug_img = toMat4b(filtered_image.rgb());
    MeshRenderer renderer(filtered_image.rgb().cols, filtered_image.rgb().rows);
    renderer.setMesh(mesh);
    renderer.setPose(pose3d);
    renderer.renderToImage(debug_img, MeshRenderer::WIREFRAME);
    imwrite("/tmp/debug_object.png", toMat3b(debug_img));
#endif

    MeshGenerator generator;
    generator.setUseColor(true);
    generator.generate(image);
    m_scene_mesh = generator.mesh();

    mesh.applyTransform(pose3d);
    RelativePoseEstimatorTableTop pose_estimator;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    meshToPointCloud(*model_cloud, mesh);

    // Refine with ICP.
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // if (first_match.matchedPoints().size() > 100)
    //     vectorToPointCloud(*scene_cloud, first_match.matchedPoints());
    // else
    rgbdImageToPointCloud(*scene_cloud, filtered_image);

#if 0
    pose_estimator.setTargetCloud(scene_cloud); // FIXME: Find out whether the removal of the (deep-copying) scene_cloud.makeShared() call sped things up.
    pose_estimator.setSourceCloud(model_cloud); // FIXME: Find out whether the removal of the (deep-copying) model_cloud.makeShared() call sped things up.
    pose_estimator.estimateNewPose();
    Pose3D mesh_transform = pose_estimator.estimatedSourcePose();
    mesh.applyTransform(mesh_transform);
#endif

    m_object_mesh = mesh;
    m_object_mesh.colors.clear();
    m_object_mesh.colors.resize(m_object_mesh.vertices.size(), cv::Vec3b(255,0,0));
    updateView();
    tc_object.stop();
}

void ObjectWindow :: updateView()
{
    ui->mesh_view->addMesh(m_scene_mesh, Pose3D(), MeshViewer::FLAT);
    ui->mesh_view->addMesh(m_table_mesh, Pose3D(), MeshViewer::FLAT);
    if (m_show_detection)
        ui->mesh_view->addMesh(m_object_mesh, Pose3D(), MeshViewer::WIREFRAME);
    ui->mesh_view->swapScene();
}

void ObjectWindow::on_detectButton_clicked()
{
    if (!m_object_finder)
        initializeDetector();

    processNewFrame(m_controller.lastImage());
}

void ObjectWindow::initializeDetector()
{
    QMutexLocker locker(&m_lock);
    resetDetector();

    ObjectFinderParams params;
    params.detection_threshold = -15;
    params.feature = SiftParameters();
    params.object_database = ui->databasePathLineEdit->text().toStdString();
    params.use_tracking = false;
    //params.object_detector = "fpfh";
    params.object_detector = "sift";
    params.keep_only_best_match = true;
    m_object_finder = ObjectFinderPtr(new ObjectFinder());
    m_object_finder->initialize(params);
}


void ObjectWindow::resetDetector()
{
    QMutexLocker locker(&m_lock);
    m_object_finder = ObjectFinderPtr();
    m_current_model_index = 0;
}

void ObjectWindow::on_databasePathLineEdit_editingFinished()
{
    resetDetector();
}

void ObjectWindow::on_browseButton_clicked()
{
    QString dirname = QFileDialog::getExistingDirectory(this,
                                                         "Select database directory...",
                                                         ui->databasePathLineEdit->text());
    if (dirname.isEmpty())
        return;

    ui->databasePathLineEdit->setText(dirname);
    resetDetector();
}

void ObjectWindow::on_showObjectCheckBox_toggled(bool checked)
{
    m_show_detection = checked;
    updateView();
}
