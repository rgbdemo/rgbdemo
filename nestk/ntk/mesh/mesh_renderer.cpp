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

#ifdef USE_GLEW
# include <GL/glew.h>
#endif

#include "mesh_renderer.h"

#include <ntk/numeric/utils.h>
#include <ntk/geometry/pose_3d.h>

using namespace cv;

namespace ntk
{

  MeshRenderer :: MeshRenderer(const Mesh& mesh, int image_width, int image_height, float transparency)
    : m_mesh(mesh), m_transparency(transparency)
  {
    m_depth_buffer = cv::Mat1f(Size(image_width, image_height));
    m_color_buffer = cv::Mat4b(Size(image_width, image_height));
    m_context = new QGLContext(QGLFormat(QGL::SampleBuffers|QGL::DepthBuffer|QGL::AlphaChannel));
    m_pbuffer = new QGLPixelBuffer(QSize(image_width, image_height), m_context->format());
    m_pbuffer->makeCurrent();
    m_list_index = glGenLists(1);

    m_pbuffer->makeCurrent();
    glEnable(GL_DEPTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearColor(1.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GLuint texture;
    if (mesh.texture.data)
    {
      glGenTextures( 1, &texture );
      glBindTexture( GL_TEXTURE_2D, texture );
      glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
      glTexImage2D(GL_TEXTURE_2D, 0,
                   GL_RGB8, mesh.texture.cols, mesh.texture.rows,
                   0, GL_BGR, GL_UNSIGNED_BYTE,mesh.texture.data);
      glEnable(GL_TEXTURE_2D);
    }

    cv::RNG rgen;
    glNewList(m_list_index, GL_COMPILE);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < mesh.faces.size(); ++i)
    {
      int v1 = mesh.faces[i].indices[0];
      int v2 = mesh.faces[i].indices[1];
      int v3 = mesh.faces[i].indices[2];
      const Point3f& p1 = mesh.vertices[v1];
      const Point3f& p2 = mesh.vertices[v2];
      const Point3f& p3 = mesh.vertices[v3];

      Vec3b color (255,255,255);

      // glColor3f(rgen.uniform(),rgen.uniform(),rgen.uniform());
      if (mesh.hasColors())
        color = mesh.colors[v1];
      glColor4f(color[0]/255.0, color[1]/255.0, color[2]/255.0, m_transparency);
      if (mesh.hasNormals())
        glNormal3f(mesh.normals[v1].x, mesh.normals[v1].y, mesh.normals[v1].z);
      if (mesh.hasTexcoords())
        glTexCoord2f(mesh.texcoords[v1].x, mesh.texcoords[v1].y);
      glVertex3f(p1.x, p1.y, p1.z);

      if (mesh.hasColors())
        color = mesh.colors[v2];
      glColor4f(color[0]/255.0, color[1]/255.0, color[2]/255.0, m_transparency);
      if (mesh.hasNormals())
        glNormal3f(mesh.normals[v2].x, mesh.normals[v2].y, mesh.normals[v2].z);
      if (mesh.hasTexcoords())
        glTexCoord2f(mesh.texcoords[v2].x, mesh.texcoords[v2].y);
      glVertex3f(p2.x, p2.y, p2.z);

      if (mesh.hasColors())
        color = mesh.colors[v3];
      glColor4f(color[0]/255.0, color[1]/255.0, color[2]/255.0, m_transparency);
      if (mesh.hasNormals())
        glNormal3f(mesh.normals[v3].x, mesh.normals[v3].y, mesh.normals[v3].z);
      if (mesh.hasTexcoords())
        glTexCoord2f(mesh.texcoords[v3].x, mesh.texcoords[v3].y);
      glVertex3f(p3.x, p3.y, p3.z);
    }
    glEnd();
    glEndList();
  }

  MeshRenderer :: ~MeshRenderer()
  {
    delete m_context;
    delete m_pbuffer;
  }

  void MeshRenderer :: estimateOptimalPlanes(const Pose3D& pose, double* near_plane, double* far_plane)
  {
    float min_z = std::numeric_limits<float>::max();
    float max_z = 0.01;

    for (int i = 0; i < m_mesh.faces.size(); ++i)
    {
      const Point3f& v1 = m_mesh.vertices[m_mesh.faces[i].indices[0]];
      const Point3f& v2 = m_mesh.vertices[m_mesh.faces[i].indices[1]];
      const Point3f& v3 = m_mesh.vertices[m_mesh.faces[i].indices[2]];

      Point3f pv1 = pose.cameraTransform(v1);
      Point3f pv2 = pose.cameraTransform(v2);
      Point3f pv3 = pose.cameraTransform(v3);

      min_z = ntk::math::min(min_z,-pv1.z);
      min_z = ntk::math::min(min_z,-pv2.z);
      min_z = ntk::math::min(min_z,-pv3.z);

      max_z = ntk::math::max(max_z,-pv1.z);
      max_z = ntk::math::max(max_z,-pv2.z);
      max_z = ntk::math::max(max_z,-pv3.z);
    }

    ntk_dbg_print(min_z, 2);
    ntk_dbg_print(max_z, 2);

    if (min_z < 0)
      min_z = 0.01;

    if (max_z < min_z)
      max_z = (min_z*2);

    *near_plane = min_z*0.9;
    *far_plane = max_z*1.1;
  }

  void MeshRenderer :: computeDepthBuffer()
  {
    glReadPixels(0, 0, m_depth_buffer.cols, m_depth_buffer.rows, GL_DEPTH_COMPONENT, GL_FLOAT, m_depth_buffer.data);
    cv::Mat1f flipped;
    flip(m_depth_buffer, flipped, 0);
    m_depth_buffer = flipped;

    // FIXME: this is very slow !!!
    // gluUnproject is not necessary, or at least one
    // could invert the projection Matrix only once.

    cv::Mat_<GLdouble> modelMatrix(4,4);
    setIdentity(modelMatrix);

    cv::Mat_<GLdouble> projMatrix(4,4);
    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix[0]);
    // projMatrix = projMatrix.inv();

    int viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);

    GLdouble objx, objy, objz;

    for (int r = 0; r < m_depth_buffer.rows; ++r)
    for (int c = 0; c < m_depth_buffer.cols; ++c)
    {
      double depth = m_depth_buffer(r,c);
      if (ntk::flt_eq(depth,1) || ntk::flt_eq(depth,0))
      {
        m_depth_buffer(r,c) = 0;
        continue;
      }
      gluUnProject(c, r, depth, modelMatrix[0], projMatrix[0], viewport,&objx, &objy, &objz);
      // double winz = (2.0*depth)-1;
      // double objz = (projMatrix(2,3)) / (winz * projMatrix(3,2) + projMatrix(3,3));
      // double objz = ;
      m_depth_buffer(r,c) = -objz;
    }
  }

  void MeshRenderer :: computeProjectionMatrix(cv::Mat4b& image, const Pose3D& pose)
  {
    double near_plane, far_plane;
    estimateOptimalPlanes(pose, &near_plane, &far_plane);
    ntk_dbg_print(near_plane, 2);
    ntk_dbg_print(far_plane, 2);
    m_last_near_plane = near_plane;
    m_last_far_plane = far_plane;

    m_pbuffer->makeCurrent();
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();
    cv::Vec3f euler_angles = pose.cvEulerRotation();
    glTranslatef(pose.cvTranslation()[0], pose.cvTranslation()[1], pose.cvTranslation()[2]);
    glRotatef(euler_angles[2]*180.0/M_PI, 0, 0, 1);
    glRotatef(euler_angles[1]*180.0/M_PI, 0, 1, 0);
    glRotatef(euler_angles[0]*180.0/M_PI, 1, 0, 0);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    double dx = pose.imageCenterX() - (image.cols / 2.0);
    double dy = pose.imageCenterY() - (image.rows / 2.0);
    glViewport(dx, -dy, image.cols, image.rows);
    //glViewport(0, 0, image.cols, image.rows);
    if (pose.isOrthographic())
    {
      gluOrtho2D(-pose.focalX()/2, pose.focalX()/2, -pose.focalY()/2, pose.focalY()/2);
    }
    else
    {
      double fov = (180.0/M_PI) * 2.0*atan(image.rows/(2.0*pose.focalY()));
      // double fov2 = (180.0/M_PI) * 2.0*atan(image.cols/(2.0*pose.focalX()));
      // ntk_dbg_print(fov2, 2);
      // gluPerspective(fov2,  double(image.rows)/image.cols, near_plane, far_plane);
      gluPerspective(fov, double(image.cols)/image.rows, near_plane, far_plane);
    }

    glMatrixMode (GL_MODELVIEW);
  }

  void MeshRenderer :: renderToImage(cv::Mat4b& image, const Pose3D& pose, int flags)
  {     
    m_pbuffer->makeCurrent();

    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();
    if (flags & LIGHTING)
    {
      GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
      GLfloat mat_shininess[] = { 10.0 };
      GLfloat light_position[] = { 1, 1, 1.0, 0.0 };
      glEnable(GL_LIGHTING);
      glShadeModel (GL_SMOOTH);
      glEnable(GL_COLOR_MATERIAL);
      glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
      glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
      glLightfv(GL_LIGHT0, GL_POSITION, light_position);
      glEnable(GL_LIGHT0);
    }

    computeProjectionMatrix(image, pose);

    glClearColor(1.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (flags & WIREFRAME)
      glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

    // draw the display list
    glCallList(m_list_index);
    glFlush();

    computeDepthBuffer();
    QImage qimage = m_pbuffer->toImage();
    for (int r = 0; r < qimage.height(); ++r)
    for (int c = 0; c < qimage.width(); ++c)
    {
      QRgb pixel = qimage.pixel(c,r);
      Vec4b color (qBlue(pixel), qGreen(pixel), qRed(pixel), qAlpha(pixel));
      m_color_buffer(r,c) = color;
      float a = qAlpha(pixel)/255.f;
      if (a > 0)
      {
        Vec4b old_color = image(r,c);
        image(r,c) = Vec4b(old_color[0]*(1-a) + color[0]*a,
                           old_color[1]*(1-a) + color[1]*a,
                           old_color[2]*(1-a) + color[2]*a,
                           255);
      }
    }
  }

} // ntk
