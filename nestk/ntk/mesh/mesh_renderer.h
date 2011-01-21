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

#ifndef MESH_RENDERER_H
#define MESH_RENDERER_H

#include "mesh.h"

#include <QtOpenGL/QGLPixelBuffer>

namespace ntk
{

class Pose3D;

  class MeshRenderer
  {
  public:
    // Flags
    enum { NORMAL = 0, WIREFRAME = 1, RANDOM_COLORS = 2, TRANSPARENCY = 4, LIGHTING = 8 };

  public:
    MeshRenderer(const Mesh& mesh, int image_width, int image_height, float transparency = 1.0);
    ~MeshRenderer();

  public:
    const cv::Mat1f& depthBuffer() const { return m_depth_buffer; }
    const cv::Mat4b& colorBuffer() const { return m_color_buffer; }

  public:
    void renderToImage(cv::Mat4b& image, const Pose3D& pose, int flags);
    void setTransparency(float f) { m_transparency = f; }

  protected:
    void computeDepthBuffer();
    void computeProjectionMatrix(cv::Mat4b& image, const Pose3D& pose);
    void estimateOptimalPlanes(const Pose3D& pose, double* near_plane, double* far_plane);

  private:
    const Mesh& m_mesh;
    QGLContext* m_context;
    QGLPixelBuffer* m_pbuffer;
    GLuint m_list_index;
    cv::Mat1f m_depth_buffer;
    cv::Mat4b m_color_buffer;
    double m_last_near_plane;
    double m_last_far_plane;
    float m_transparency;
  };

} // ntk

#endif // MESH_RENDERER_H
