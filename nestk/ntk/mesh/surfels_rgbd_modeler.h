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

#ifndef NTK_MESH_SURFELS_RGBD_MODELER_H
#define NTK_MESH_SURFELS_RGBD_MODELER_H

#include <ntk/core.h>
#include <ntk/camera/calibration.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/rgbd_modeler.h>

namespace ntk
{

class SurfelsRGBDModeler : public RGBDModeler
{
public:
  SurfelsRGBDModeler() : m_min_views(2)
  {}

public:
  void setMinViewsPerSurfel(int n) { m_min_views = n; }

public:
  virtual void addNewView(const RGBDImage& image, Pose3D& relative_pose);
  virtual void computeMesh();

  virtual void reset() { RGBDModeler::reset(); m_surfels.clear(); }

protected:
  std::vector<Surfel> m_surfels;
  int m_min_views;
};

class ICPSurfelsRGBDModeler : public SurfelsRGBDModeler
{
public:
  ICPSurfelsRGBDModeler() : SurfelsRGBDModeler()
  {}

public:
  void setMinViewsPerSurfel(int n) { m_min_views = n; }

public:
  virtual void addNewView(const RGBDImage& image, Pose3D& relative_pose);

protected:
  Pose3D fixRelativePose(const RGBDImage& image, const Pose3D& relative_pose);

private:
  ntk::Mesh m_point_cloud;
};

} // ntk

#endif // NTK_MESH_SURFELS_RGBD_MODELER_H
