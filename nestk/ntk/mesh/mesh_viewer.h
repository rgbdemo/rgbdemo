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

#ifndef MESHVIEWER_H
#define MESHVIEWER_H

#include <ntk/core.h>
#include <ntk/mesh/mesh.h>

#include <QGLWidget>
#include <QPoint>

namespace ntk
{

class Pose3D;

class MeshViewer : public QGLWidget
{
  Q_OBJECT

public:
  enum MeshViewerMode { FLAT = 1, WIREFRAME = 2 };

public:
  MeshViewer(QWidget *parent = 0)
    : QGLWidget(parent),
    m_mesh_center(0,0,0),
    m_use_vertex_buffer_object(false)
  {}

  void addMesh(const ntk::Mesh& mesh, const Pose3D& pose, MeshViewerMode mode);
  void addMeshToDisplayList(const ntk::Mesh& mesh, const Pose3D& pose, MeshViewerMode mode);
  void addMeshToVertexBufferObject(const ntk::Mesh& mesh, const Pose3D& pose, MeshViewerMode mode);
  void addPlane(const ntk::Plane& plane);
  void swapScene();
  void setVertexBufferObjectMode(bool enable) { m_use_vertex_buffer_object = enable; }

  void resetCamera();
  void rotateCamera(const cv::Vec3f& axis,
                    double angle);
  void setCameraLookat(const cv::Vec3f& eye,
                       const cv::Vec3f& center,
                       const cv::Vec3f& up);

  void enableLighting();

protected:
  virtual void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void  updateDisplayCenter();

protected:
  void mousePressEvent(QMouseEvent *);
  void mouseMoveEvent(QMouseEvent*);

private:
  struct VertexBufferObject {
    unsigned vertex_id;
    unsigned faces_id;
    int color_offset;
    int texture_offset;
    int nb_vertices;
    int nb_faces;
    bool has_texcoords;
    bool has_color;
    bool has_faces;
  };

private:
  QPoint m_last_mouse_pos;
  std::vector<VertexBufferObject> m_vertex_buffer_objects;
  std::vector<VertexBufferObject> m_upcoming_vertex_buffer_objects;
  std::vector<int> m_display_lists;
  std::vector<int> m_upcoming_display_lists;
  std::vector<GLuint> m_textures;
  std::vector<GLuint> m_upcoming_textures;
  cv::Point3f m_mesh_center;
  cv::Point3f m_display_center;
  bool m_use_vertex_buffer_object;
};

} // ntk

#endif // MESHVIEWER_H
