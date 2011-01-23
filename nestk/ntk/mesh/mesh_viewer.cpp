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

#define GL_GLEXT_PROTOTYPES

#ifdef USE_GLEW
#include <GL/glew.h>
#else
// not required, qt opengl includes it
// avoids problems on Mac
// #include <GL/gl.h>
#endif

#include "mesh_viewer.h"

#include <ntk/ntk.h>
#include <ntk/geometry/pose_3d.h>

#include <QMouseEvent>


using namespace cv;

namespace ntk
{

void MeshViewer :: initializeGL()
{
#ifdef USE_GLEW
  GLenum err = glewInit();
  ntk_ensure(err == GLEW_OK, "Could not load GLEW.");
  std::cout << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
  if (GLEW_ARB_vertex_buffer_object)
    m_use_vertex_buffer_object = true;
#else
  m_use_vertex_buffer_object = false;
#endif
  ntk_dbg_print(m_use_vertex_buffer_object, 1);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LINE_SMOOTH);
  glClearColor(0.0f, 0.0f, 0.2f, 1.0f);
  resetCamera();
}

void MeshViewer :: enableLighting()
{
  makeCurrent();
  glEnable(GL_LIGHTING);
  GLfloat specular[] = {1.0f, 1.0f, 1.0f , 1.0f};
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  GLfloat position[] = { 0.1f, 1.0f, 0.5f, 1.0f };
  glLightfv(GL_LIGHT0, GL_POSITION, position);
  glEnable(GL_LIGHT0);
  glEnable(GL_SMOOTH);
  glShadeModel(GL_FLAT);
  glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
  glEnable ( GL_COLOR_MATERIAL );
}

void MeshViewer :: addMeshToDisplayList(const ntk::Mesh& mesh, const Pose3D& pose, MeshViewerMode mode)
{
  int new_list_index = glGenLists(1);
  glNewList(new_list_index, GL_COMPILE);
  if (mesh.texture.data)
  {
    // Last texture id was just created
    GLuint texture = m_upcoming_textures[m_upcoming_textures.size()-1];
    glEnable(GL_TEXTURE_2D);
    glBindTexture( GL_TEXTURE_2D, texture );
  }
  else
  {
    glDisable(GL_TEXTURE_2D);
  }
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(pose.cvTranslation()[0], pose.cvTranslation()[1], pose.cvTranslation()[2]);
  Vec3f euler_angles = pose.cvEulerRotation();
  glRotatef(rad_to_deg(euler_angles[0]), 1, 0, 0);
  glRotatef(rad_to_deg(euler_angles[1]), 0, 1, 0);
  glRotatef(rad_to_deg(euler_angles[2]), 0, 0, 1);

  if (mode & WIREFRAME)
  {
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
  }
  else
  {
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  }

  int64 point_start_time = ntk::Time::getMillisecondCounter();
  if (mesh.faces.size() == 0)
  {
    glBegin(GL_POINTS);
    for (int i = 0; i < mesh.vertices.size(); ++i)
    {
      const Point3f& v = mesh.vertices[i];
      // if (i % 1000 == 0)
      // ntk_dbg_print(v, 1);
      if (mesh.hasColors())
        glColor3f(mesh.colors[i][0]/255.0, mesh.colors[i][1]/255.0, mesh.colors[i][2]/255.0);
      glVertex3f(v.x, v.y, v.z);
    }
    glEnd();
  }
  int64 point_end_time = ntk::Time::getMillisecondCounter();
  ntk_dbg_print(point_end_time-point_start_time, 1);

  {
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < mesh.faces.size(); ++i)
    {
      int i1 = mesh.faces[i].indices[0];
      int i2 = mesh.faces[i].indices[1];
      int i3 = mesh.faces[i].indices[2];

      const Point3f& v1 = mesh.vertices[i1];
      const Point3f& v2 = mesh.vertices[i2];
      const Point3f& v3 = mesh.vertices[i3];

      Vec3f nm = (Vec3f(v2-v1).cross(v3-v2));
      normalize(nm);

      if (!mesh.hasColors())
        glColor3f(1.0f, 0.0f, 0.0f);

      if (mesh.hasColors())
        glColor3f(mesh.colors[i1][0]/255.0, mesh.colors[i1][1]/255.0, mesh.colors[i1][2]/255.0);
      if (mesh.hasTexcoords())
        glTexCoord2f(mesh.texcoords[i1].x, mesh.texcoords[i1].y);
      glVertex3f(v1.x, v1.y, v1.z);
      glNormal3f(nm[0], nm[1], nm[2]);

      if (mesh.hasColors())
        glColor3f(mesh.colors[i2][0]/255.0, mesh.colors[i2][1]/255.0, mesh.colors[i2][2]/255.0);
      if (mesh.hasTexcoords())
        glTexCoord2f(mesh.texcoords[i2].x, mesh.texcoords[i2].y);
      glVertex3f(v2.x, v2.y, v2.z);
      glNormal3f(nm[0], nm[1], nm[2]);

      if (mesh.hasColors())
        glColor3f(mesh.colors[i3][0]/255.0, mesh.colors[i3][1]/255.0, mesh.colors[i3][2]/255.0);
      if (mesh.hasTexcoords())
        glTexCoord2f(mesh.texcoords[i3].x, mesh.texcoords[i3].y);
      glVertex3f(v3.x, v3.y, v3.z);
      glNormal3f(nm[0], nm[1], nm[2]);
    }
    glEnd();
  }
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glEndList();

  m_upcoming_display_lists.push_back(new_list_index);
}

void MeshViewer :: addMeshToVertexBufferObject(const ntk::Mesh& mesh, const Pose3D& pose, MeshViewerMode mode)
{
#ifdef USE_GLEW
  GLuint vbo_id = -1, vbo_faces_id = -1;
  glGenBuffersARB(1, &vbo_id);
  if (mesh.hasFaces())
    glGenBuffersARB(1, &vbo_faces_id);

  VertexBufferObject vbo;
  vbo.nb_faces = 0;
  vbo.vertex_id = vbo_id;
  vbo.faces_id = vbo_faces_id;
  vbo.has_faces = mesh.hasFaces();
  vbo.has_color = mesh.hasColors();
  vbo.has_texcoords = mesh.hasTexcoords();
  vbo.color_offset = mesh.vertices.size()*sizeof(Vec3f);
  vbo.texture_offset = mesh.vertices.size()*sizeof(Vec3f) + mesh.colors.size() * sizeof(Vec3b);
  vbo.nb_vertices = mesh.vertices.size();

  glBindBufferARB(GL_ARRAY_BUFFER_ARB, vbo.vertex_id);
  glBufferDataARB(GL_ARRAY_BUFFER_ARB,
                  mesh.colors.size()*sizeof(Vec3b)
                  + mesh.vertices.size()*sizeof(Vec3f)
                  + mesh.texcoords.size()*sizeof(Point2f), // size
                  0, // null pointer: just allocate memory
                  GL_STATIC_DRAW_ARB);
  glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, 0, mesh.vertices.size()*sizeof(Vec3f), &mesh.vertices[0]);
  if (vbo.has_color)
    glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, vbo.color_offset, mesh.colors.size()*sizeof(Vec3b), &mesh.colors[0]);
  if (vbo.has_texcoords)
    glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, vbo.texture_offset, mesh.texcoords.size()*sizeof(Point2f), &mesh.texcoords[0]);

  if (vbo.has_faces)
  {
    vbo.nb_faces = mesh.faces.size();
    glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, vbo.faces_id);
    glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,
                    mesh.faces.size() * 3 * sizeof(GLuint), // size
                    (GLuint*)&mesh.faces[0],
                    GL_STATIC_DRAW_ARB);
  }
  m_upcoming_vertex_buffer_objects.push_back(vbo);
#endif
}

void MeshViewer :: addMesh(const ntk::Mesh& mesh, const Pose3D& pose, MeshViewerMode mode)
{  
  unsigned long start = ntk::Time::getMillisecondCounter();
  makeCurrent();

  // Compute new mesh center.
  {
    m_mesh_center = Point3f(0,0,0);
    int n_sample = 0;
    for (int i = 0; i < mesh.vertices.size(); i += mesh.vertices.size()/100 + 1)
    {
      m_mesh_center += pose.cameraTransform(mesh.vertices[i]);
      ++n_sample;
    }
    if (mesh.vertices.size()>0)
      m_mesh_center *= 1.0/n_sample;
  }

  // Setup texture
  GLuint texture;
  if (mesh.texture.data)
  {
    glGenTextures( 1, &texture );
    m_upcoming_textures.push_back(texture);
    glBindTexture( GL_TEXTURE_2D, texture );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_BASE_LEVEL,0);
    //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAX_LEVEL,0);
    //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0,
                 GL_RGB8, mesh.texture.cols, mesh.texture.rows,
                 0, GL_BGR, GL_UNSIGNED_BYTE,mesh.texture.data);
  }

  if (m_use_vertex_buffer_object)
  {
    addMeshToVertexBufferObject(mesh, pose, mode);
  }
  else
  {
    addMeshToDisplayList(mesh, pose, mode);
  }

  unsigned long end = ntk::Time::getMillisecondCounter();
  //ntk_dbg_print((end-start) / 1000., 1);
  updateDisplayCenter();  
}

void MeshViewer :: swapScene()
{
#ifdef USE_GLEW
    foreach_idx(i, m_vertex_buffer_objects) {
    GLuint vboId = m_vertex_buffer_objects[i].vertex_id;
    glDeleteBuffersARB(1, &vboId);
    if (m_vertex_buffer_objects[i].has_faces)
    {
      vboId = m_vertex_buffer_objects[i].faces_id;
      glDeleteBuffersARB(1, &vboId);
    }
  }
#endif

  foreach_idx(i, m_display_lists) {
    glDeleteLists(m_display_lists[i], 1);
  }

  foreach_idx(i, m_textures) {
    glDeleteTextures(1, &m_textures[i]);
  }

  m_vertex_buffer_objects.clear();
  m_display_lists.clear();
  m_textures.clear();

  m_display_lists = m_upcoming_display_lists;
  m_textures = m_upcoming_textures;
  m_vertex_buffer_objects = m_upcoming_vertex_buffer_objects;
  m_upcoming_textures.clear();
  m_upcoming_display_lists.clear();
  m_upcoming_vertex_buffer_objects.clear();
  updateGL();
}

void MeshViewer :: resizeGL(int w, int h)
{
  glViewport(0, 0, (GLint)w, (GLint)h);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(50.0, double(w)/h, 0.01, 20);
}

void MeshViewer :: resetCamera()
{
  makeCurrent();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0,0,0,0,0,-5,0,1,0);
  updateDisplayCenter();
}

void MeshViewer :: setCameraLookat(const cv::Vec3f& eye,
                                   const cv::Vec3f& center,
                                   const cv::Vec3f& up)
{
  makeCurrent();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(eye[0],eye[1],eye[2],
            center[0],center[1],center[2],
            up[0], up[1], up[2]);
  updateDisplayCenter();
}

void MeshViewer :: mousePressEvent(QMouseEvent* event)
{
  m_last_mouse_pos = event->pos();
}

void MeshViewer :: rotateCamera(const cv::Vec3f& axis,
                                double angle)
{
  GLdouble m[16];
  makeCurrent();
  glMatrixMode(GL_MODELVIEW);
  glGetDoublev(GL_MODELVIEW_MATRIX, m);
  glLoadIdentity();
  glTranslatef(m_display_center.x,m_display_center.y,m_display_center.z);
  glRotatef(angle, axis[0], axis[1], axis[2]);
  glTranslatef(-m_display_center.x,-m_display_center.y,-m_display_center.z);
  glMultMatrixd(m);
  updateDisplayCenter();
}

void MeshViewer :: mouseMoveEvent(QMouseEvent* event)
{
  GLdouble m[16];
  QPoint pos = event->pos();
  if (event->buttons() & Qt::RightButton)
  {
    double dy = pos.y() - m_last_mouse_pos.y();
    makeCurrent();
    glMatrixMode(GL_MODELVIEW);
    glGetDoublev(GL_MODELVIEW_MATRIX, m);
    glLoadIdentity();
    glTranslatef(0,0,-dy*0.01);
    glMultMatrixd(m);
  }
  else if (event->buttons() & Qt::LeftButton)
  {
    double dx = pos.x() - m_last_mouse_pos.x();
    double dy = pos.y() - m_last_mouse_pos.y();
    makeCurrent();
    glMatrixMode(GL_MODELVIEW);
    glGetDoublev(GL_MODELVIEW_MATRIX, m);
    glLoadIdentity();
    glTranslatef(m_display_center.x,m_display_center.y,m_display_center.z);
    glRotatef(dx*0.2, 0,1,0);
    glRotatef(dy*0.2, 1,0,0);
    glTranslatef(-m_display_center.x,-m_display_center.y,-m_display_center.z);
    glMultMatrixd(m);
  }
  else if (event->buttons() & Qt::MidButton)
  {
    double dx = pos.x() - m_last_mouse_pos.x();
    double dy = pos.y() - m_last_mouse_pos.y();
    makeCurrent();
    glMatrixMode(GL_MODELVIEW);
    glGetDoublev(GL_MODELVIEW_MATRIX, m);
    glLoadIdentity();
    glTranslatef(dx*0.01,-dy*0.01,0);
    glMultMatrixd(m);
  }

  m_last_mouse_pos = pos;
  updateDisplayCenter();
  updateGL();
}

void MeshViewer :: updateDisplayCenter()
{
  makeCurrent();
  GLdouble m[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, m);
  cv::Mat1d cv_mat(4, 4, m);
  cv::Mat1d cv_p (4,1);
  cv_p(0,0) = m_mesh_center.x; cv_p(1,0) = m_mesh_center.y;  cv_p(2,0) = m_mesh_center.z; cv_p(3,0) = 1;
  cv_p = cv_mat.t() * cv_p;
  m_display_center = Point3f(cv_p(0,0), cv_p(1,0), cv_p(2,0));
}

void MeshViewer :: paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  unsigned long start = ntk::Time::getMillisecondCounter();

#ifdef USE_GLEW
  if (m_use_vertex_buffer_object)
  {
    foreach_idx(i, m_vertex_buffer_objects)
    {
      glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_vertex_buffer_objects[i].vertex_id);

      if (m_vertex_buffer_objects[i].has_texcoords)
      {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, m_textures[i]);
      }
      else
      {
        glDisable(GL_TEXTURE_2D);
      }

      if (m_vertex_buffer_objects[i].has_color)
        glEnableClientState(GL_COLOR_ARRAY);
      if (m_vertex_buffer_objects[i].has_texcoords)
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
      glEnableClientState(GL_VERTEX_ARRAY);

      glVertexPointer(3, GL_FLOAT, 0, 0);
      if (m_vertex_buffer_objects[i].has_color)
        glColorPointer(3, GL_UNSIGNED_BYTE, 0, ((char*) NULL) + m_vertex_buffer_objects[i].color_offset);

      if (m_vertex_buffer_objects[i].has_texcoords)
        glTexCoordPointer(2, GL_FLOAT, 0, ((char*) NULL) + m_vertex_buffer_objects[i].texture_offset);

      if (m_vertex_buffer_objects[i].has_faces)
      {
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, m_vertex_buffer_objects[i].faces_id);
        glNormal3f(0, 0, 1);
        glDrawElements(GL_TRIANGLES, m_vertex_buffer_objects[i].nb_faces*3, GL_UNSIGNED_INT, 0);
      }
      else
      {
        glDrawArrays(GL_POINTS,
                     0,
                     m_vertex_buffer_objects[i].nb_vertices);
      }

      glDisableClientState(GL_VERTEX_ARRAY);

      if (m_vertex_buffer_objects[i].has_color)
        glDisableClientState(GL_COLOR_ARRAY);

      if (m_vertex_buffer_objects[i].has_texcoords)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);

      // bind with 0, so, switch back to normal pointer operation
      glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

      if (m_vertex_buffer_objects[i].has_faces)
      {
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
      }
    }
  }
  else
#endif
  {
    foreach_idx(i, m_display_lists)
    {
      glCallList(m_display_lists[i]);
    }
    glFlush();
  }

  unsigned long end = ntk::Time::getMillisecondCounter();
 // ntk_dbg_print((end-start) / 1000., 1);
}

} // ntk
