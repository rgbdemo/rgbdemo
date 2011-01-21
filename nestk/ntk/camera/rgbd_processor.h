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

#ifndef NTK_CAMERA_RGBD_PROCESSOR_H
#define NTK_CAMERA_RGBD_PROCESSOR_H

#include <ntk/core.h>
#include <opencv/cv.h>
#include <ntk/camera/rgbd_image.h>

namespace ntk
{

  class RGBDProcessor
  {
  public:
    enum ProcessorFlag {
      NoProcessing = 0x0,
      FixGeometry = 0x1, // transform euclidian distance to depth (z component)
      UndistortImages = 0x2, // apply undistort
      FixBias = 0x4, // fix depth bias using polynomials
      FilterAmplitude = 0x8, // remove low amplitude pixels
      FilterNormals = 0x10, // remove normals not aligned enough with line of sight
      FilterUnstable = 0x20, // remove depth values changing too much between consecutive frames
      FilterEdges = 0x40, // remove pixels with high depth spatial derivative
      FilterThresholdDepth = 0x80, // set a depth range
      FilterMedian = 0x100,      
      ComputeMapping = 0x200, // compute mappings between rgb and depth
      ComputeNormals = 0x400, // compute normals (required by FilterNormals)
      ComputeKinectDepthTanh = 0x800, // compute depth in meters from kinect values
      ComputeKinectDepthLinear = 0x1000, // compute depth in meters from kinect values
      ComputeKinectDepthBaseline = 0x2000, // compute depth in meters from kinect values
      NoAmplitudeIntensityUndistort = 0x4000, // apply undistort
      Pause = 0x8000,
      RemoveSmallStructures = 0x10000,
      FillSmallHoles = 0x20000,
      FlipColorImage = 0x40000
    };

  public:
    RGBDProcessor() :
        m_image(0),
        //m_flags(FixGeometry | FixBias | UndistortImages | ComputeNormals),
        m_flags(UndistortImages),
        m_min_depth(0.3),
        m_max_depth(2.0),
        m_max_normal_angle(80),
        m_max_time_depth_delta(0.1),
        m_max_spatial_depth_delta(0.1),
        m_mapping_resolution(1.0)
    {}

  public:
    bool hasFilterFlag(ProcessorFlag flag) const { return m_flags&flag; }
    void setFilterFlags(int flags) { m_flags = flags; }
    void setFilterFlag(ProcessorFlag flag, bool enabled)
    { if (enabled) m_flags |= flag; else m_flags &= ~flag; }

    void setMinDepth(float meters) { m_min_depth = meters; }
    float minDepth() const { return m_min_depth; }
    void setMaxDepth(float meters) { m_max_depth = meters; }
    float maxDepth() const { return m_max_depth; }
    void setMaxNormalAngle(float angle_in_deg) { m_max_normal_angle = angle_in_deg; }
    void setMaxTimeDelta(float v) { m_max_time_depth_delta = v; }
    void setMaxSpacialDelta(float v) { m_max_spatial_depth_delta = v; }
    void setMappingResolution(float r) { m_mapping_resolution = r; }

  public:
    virtual void processImage(RGBDImage& image);
    void undistortImages();
    void fixDepthGeometry();
    void fixDepthBias();
    void removeLowAmplitudeOutliers();
    void removeNormalOutliers();
    void removeUnstableOutliers();
    void removeEdgeOutliers();
    void applyDepthThreshold();
    void computeNormals();
    void computeMappings();
    void medianFilter();
    void computeKinectDepthLinear();
    void computeKinectDepthTanh();
    void computeKinectDepthBaseline();
    void removeSmallStructures();
    void fillSmallHoles();

  private:
    RGBDImage* m_image;
    int m_flags;
    cv::Mat1f m_last_depth_image;
    float m_min_depth;
    float m_max_depth;
    float m_max_normal_angle;
    float m_max_time_depth_delta;
    float m_max_spatial_depth_delta;
    float m_mapping_resolution;
  };

  void compute_color_encoded_depth(const cv::Mat1f& depth, cv::Mat3b& color_dept,
                                   double* min_val = 0, double* max_val = 0);

} // ntk

#endif // NTK_CAMERA_RGBD_PROCESSOR_H
