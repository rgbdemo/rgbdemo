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

#include "rgbd_processor.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/geometry/pose_3d.h>

using namespace cv;

namespace ntk
{

  void RGBDProcessor :: fixDepthGeometry()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    double cx = m_image->calibration()->depth_intrinsics(0,2);
    double cy = m_image->calibration()->depth_intrinsics(1,2);
    double fx = m_image->calibration()->depth_intrinsics(0,0);
    double fy = m_image->calibration()->depth_intrinsics(1,1);

    cv::Mat1f& depth_im = m_image->depthRef();
    for_all_rc(depth_im)
    {
      double orig_depth = depth_im(r,c);

      double dx = c-cx;
      double dy = r-cy;

      Point3f v(dx/fx, dy/fy, 1);
      double norm = sqrt(v.dot(v));
      v = v * (1.0/norm);
      v *= orig_depth;

      double depth_z = v.z;
      depth_im(r,c) = depth_z;
    }
  }

  void RGBDProcessor :: fixDepthBias()
  {
    ntk_assert(0, "not updated.");
#if 0 // FIXME: obsolete
    cv::Mat1f& depth_im = m_image->depthRef();
    for_all_rc(depth_im)
    {
      double depth_z = depth_im(r,c);

      // depth correction is accurate only in the 30cm-110cm range.
      double x = depth_z;
      if (depth_z < 0.3) x = 0.3;
      if (depth_z > 0.8) x = 0.8;
      double offset =   m_image->calibration()->depth_correction(0,4)*x*x*x*x
                      + m_image->calibration()->depth_correction(0,3)*x*x*x
                      + m_image->calibration()->depth_correction(0,2)*x*x
                      + m_image->calibration()->depth_correction(0,1)*x
                      + m_image->calibration()->depth_correction(0,0);

      depth_im(r,c) = depth_z + offset;
    }
#endif
  }

  // FIXME: why is it so slow ?
  void RGBDProcessor :: computeNormals()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    const Pose3D& depth_pose = *m_image->calibration()->depth_pose;
    const cv::Mat1f& depth_im = m_image->depth();
    cv::Mat3f& normal_im = m_image->normalRef();
    normal_im = cv::Mat3f(depth_im.size());
    normal_im = Vec3f(0,0,0);
    for_all_rc(depth_im)
    {
      normal_im(r,c) = estimate_normal_from_depth(depth_im, depth_pose, r, c);
    }
  }

  void RGBDProcessor :: processImage(RGBDImage& image)
  {
    m_image = &image;

    if (m_flags & FlipColorImage)
    {
      flip(m_image->rawRgb(), m_image->rawRgbRef(), -1);
    }

    if (m_image->rawIntensity().data)
    {
      normalize(m_image->rawIntensityRef(), m_image->rawIntensityRef(), 0, 255, NORM_MINMAX, -1);
    }

    if (!m_image->calibration() || !(m_flags & UndistortImages) || (m_flags & Pause))
    {      
      m_image->rawAmplitude().copyTo(m_image->amplitudeRef());
      m_image->rawIntensity().copyTo(m_image->intensityRef());
      m_image->rawDepth().copyTo(m_image->depthRef());
      m_image->rawRgb().copyTo(m_image->rgbRef());
    }

    TimeCount tc_undistort("undistort", 2);
    if (m_image->calibration() && (m_flags & UndistortImages))
      undistortImages();
    tc_undistort.stop();

    TimeCount tc_depth("compute_depth", 2);
    if (m_flags & ComputeKinectDepthLinear)
      computeKinectDepthLinear();
    else if (m_flags & ComputeKinectDepthTanh)
      computeKinectDepthTanh();
    else if (m_flags & ComputeKinectDepthBaseline)
      computeKinectDepthBaseline();
    tc_depth.stop();

    m_image->depthMaskRef() = cv::Mat1b(m_image->rawDepth().size());
    for_all_rc(m_image->depthMaskRef())
    {
      if (m_image->depth()(r,c) < 1e-5)
        m_image->depthMaskRef()(r,c) = 0;
      else
        m_image->depthMaskRef()(r,c) = 1;
    }

    if (m_image->calibration())
    {
      if (m_flags & FixGeometry)
        fixDepthGeometry();

      if (m_flags & FixBias)
        fixDepthBias();

      if (m_flags & FilterMedian)
        medianFilter();

      if ((m_flags & ComputeNormals) || (m_flags & FilterNormals))
        computeNormals();

      if (m_flags & FilterThresholdDepth)
        applyDepthThreshold();

      if (m_flags & FilterAmplitude)
        removeLowAmplitudeOutliers();

      if (m_flags & FilterEdges)
        removeEdgeOutliers();

      if (m_flags & FilterNormals)
        removeNormalOutliers();

      if (m_flags & FilterUnstable)
        removeUnstableOutliers();

      if (m_flags & ComputeMapping)
        computeMappings();

      if (m_flags & RemoveSmallStructures)
        removeSmallStructures();
      if (m_flags & FillSmallHoles)
        fillSmallHoles();
    }

    if (m_image->rgb().data)
      cvtColor(m_image->rgb(), m_image->rgbAsGrayRef(), CV_BGR2GRAY);
    int64 tfinal = ntk::Time::getMillisecondCounter();
  }

  void RGBDProcessor :: undistortImages()
  {
    cv::Mat3b tmp3b;
    cv::Mat1f tmp1f;

    cv::Mat3b& rgb_im = m_image->rgbRef();
    const cv::Mat3b& raw_rgb_im = m_image->rawRgb();

    if (m_image->calibration()->rgbSize() != m_image->calibration()->rawRgbSize())
    {
      // First cut color image to the undistorted image size (not used for kinect).
      cv::Size rgb_size = m_image->calibration()->rgbSize();
      cv::Mat roi = raw_rgb_im(cv::Rect((raw_rgb_im.cols-rgb_size.width)/2.0,
                                        (raw_rgb_im.rows-rgb_size.height)/2.0,
                                        rgb_size.width,
                                        rgb_size.height));
      roi.copyTo(rgb_im);
    }
    else
    {
      raw_rgb_im.copyTo(rgb_im);
    }

    if (!m_image->calibration()->zero_rgb_distortion)
    {
      remap(rgb_im, tmp3b,
            m_image->calibration()->rgb_undistort_map1,
            m_image->calibration()->rgb_undistort_map2,
            CV_INTER_LINEAR);
      tmp3b.copyTo(rgb_im);
    }

    if (!m_image->calibration()->zero_depth_distortion)
    {
      remap(m_image->rawDepthRef(), m_image->depthRef(),
            m_image->calibration()->depth_undistort_map1,
            m_image->calibration()->depth_undistort_map2,
            CV_INTER_LINEAR);
    }
    else
    {
      m_image->rawDepthRef().copyTo(m_image->depthRef());
    }

    if (m_image->calibration()->zero_depth_distortion ||
        (m_flags & NoAmplitudeIntensityUndistort))
    {
      m_image->rawAmplitudeRef().copyTo(m_image->amplitudeRef());
      m_image->rawIntensityRef().copyTo(m_image->intensityRef());
    }
    else      
    {
      remap(m_image->rawAmplitudeRef(), m_image->amplitudeRef(),
            m_image->calibration()->depth_undistort_map1,
            m_image->calibration()->depth_undistort_map2,
            CV_INTER_LINEAR);

      remap(m_image->rawIntensityRef(), m_image->intensityRef(),
            m_image->calibration()->depth_undistort_map1,
            m_image->calibration()->depth_undistort_map2,
            CV_INTER_LINEAR);
    }
  }

  void RGBDProcessor :: computeMappings()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");

    cv::Mat1b& mask_im = m_image->depthMaskRef();

    const Pose3D& depth_pose = *m_image->calibration()->depth_pose;
    const Pose3D& rgb_pose = *m_image->calibration()->rgb_pose;

    const cv::Mat1f& depth_im = m_image->depth();
    const cv::Mat3b& rgb_im = m_image->rgb();

    cv::Mat1f& mapped_depth = m_image->mappedDepthRef();
    mapped_depth = cv::Mat1f(m_image->rgb().size());
    mapped_depth = 0.f;

    cv::Mat3b& mapped_color = m_image->mappedRgbRef();
    mapped_color = cv::Mat3b(m_image->depth().size());

    float delta = 1.0 / m_mapping_resolution;
    for (float r = 0; r < depth_im.rows; r += delta )
    for (float c = 0; c < depth_im.cols; c += delta )
    {
      int i_r = ntk::math::rnd(r);
      int i_c = ntk::math::rnd(c);
      if (!is_yx_in_range(depth_im, i_r, i_c))
        continue;

      if (!mask_im(i_r, i_c))
        continue;

      double depth = depth_im(i_r,i_c);
      cv::Point3f p = depth_pose.unprojectFromImage(Point2f(c,r), depth);
      cv::Point3f prgb = rgb_pose.projectToImage(p);

      int i_y = ntk::math::rnd(prgb.y);
      int i_x = ntk::math::rnd(prgb.x);
      if (is_yx_in_range(rgb_im, i_y, i_x))
      {
        Vec3b bgr = rgb_im(i_y, i_x);
        mapped_color(i_r, i_c) = bgr;
        mapped_depth(i_y, i_x) = prgb.z;
      }
    }
  }

  void RGBDProcessor :: medianFilter()
  {
    medianBlur(m_image->depthRef(), m_image->depthRef(), 3);
  }

  void RGBDProcessor :: removeLowAmplitudeOutliers()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    cv::Mat1b& mask_im = m_image->depthMaskRef();
    const cv::Mat1f& depth_im = m_image->depth();
    double amplitude_sum = sum(m_image->amplitude())[0];

    for_all_rc(depth_im)
    {
      if (!mask_im(r,c)) continue;
      double da = m_image->amplitude()(r,c) / amplitude_sum;
      if (da < 0.02)
        mask_im(r,c) = 0;
    }
  }

  void RGBDProcessor :: removeNormalOutliers()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    const Pose3D& depth_pose = *m_image->calibration()->depth_pose;
    cv::Mat1b& mask_im = m_image->depthMaskRef();
    const cv::Mat1f& depth_im = m_image->depth();

    for_all_rc(depth_im)
    {
      if (!mask_im(r,c)) continue;
      Vec3f eyev = camera_eye_vector(depth_pose, r, c);
      Vec3f normal = m_image->normal()(r, c);
      double angle = acos(normal.dot(eyev));
      if (angle > (m_max_normal_angle*M_PI/180.0))
        mask_im(r,c) = 0;
    }
  }

  void RGBDProcessor :: removeUnstableOutliers()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    cv::Mat1b& mask_im = m_image->depthMaskRef();
    const cv::Mat1f& depth_im = m_image->depth();

    if (!m_last_depth_image.data)
    {
      m_image->depth().copyTo(m_last_depth_image);
      return;
    }

    for_all_rc(depth_im)
    {
      if (!mask_im(r,c)) continue;
      float diff = std::abs(m_last_depth_image(r,c) - depth_im(r,c));
      if (diff > m_max_time_depth_delta)
        mask_im(r,c) = 0;
    }

    m_image->depth().copyTo(m_last_depth_image);
  }

  void RGBDProcessor :: removeSmallStructures()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    cv::Mat1b& mask_im = m_image->depthMaskRef();

    cv::morphologyEx(mask_im, mask_im,
                     cv::MORPH_OPEN,
                     getStructuringElement(cv::MORPH_RECT,
                                           cv::Size(5,5)));

    cv::morphologyEx(mask_im, mask_im,
                     cv::MORPH_ERODE,
                     getStructuringElement(cv::MORPH_RECT,
                                           cv::Size(3,3)));

  }

  void RGBDProcessor :: fillSmallHoles()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    cv::Mat1b& mask_im = m_image->depthMaskRef();
    cv::Mat1f& depth_im = m_image->depthRef();

    cv::Mat1b new_mask;
    cv::morphologyEx(mask_im, new_mask,
                     cv::MORPH_CLOSE,
                     getStructuringElement(cv::MORPH_RECT,
                                           cv::Size(10,10)));
    cv::Mat1b paint_mask = new_mask - mask_im;
    mask_im = new_mask;

    for_all_rc(mask_im)
    {
      if (mask_im(r,c) && depth_im(r,c) < 1e-5)
        paint_mask(r,c) = 1;
    }

    cv::Mat1b depth8b (depth_im.size());
    for_all_rc(depth8b)
    {
      depth8b(r,c) = 255 * (depth_im(r,c) - m_min_depth) / (m_max_depth - m_min_depth);
    }

    cv::Mat1b painted_depth8b;

    cv::inpaint(depth8b, paint_mask, painted_depth8b, 10, cv::INPAINT_NS);
    imwrite("debug_depth.png", depth8b);
    imwrite("debug_painted_depth.png", painted_depth8b);
    imwrite("debug_paint_mask.png", paint_mask);
    imwrite("debug_depth_mask.png", mask_im);

    for_all_rc(paint_mask)
    {
      if (paint_mask(r,c))
      {
        double d = painted_depth8b(r,c) * (m_max_depth-m_min_depth) / 255.0 + m_min_depth;
        depth_im(r,c) = d;
      }
    }
  }

  void RGBDProcessor :: removeEdgeOutliers()
  {
    ntk_ensure(m_image->calibration(), "Calibration required.");
    cv::Mat1b& mask_im = m_image->depthMaskRef();
    const cv::Mat1f& depth_im = m_image->depth();

    for_all_rc(depth_im)
    {
      if (!mask_im(r,c)) continue;

      if (!is_yx_in_range(depth_im, r+1, c+1))
        continue;
      double diff = std::abs(depth_im(r,c) - depth_im(r,c+1))
                    + std::abs(depth_im(r,c) - depth_im(r+1,c));
      diff /= 2.0;
      if (diff > m_max_spatial_depth_delta)
      {
        mask_im(r,c) = 0;
        continue;
      }
    }
  }

  void RGBDProcessor :: computeKinectDepthTanh()
  {
    const float k1 = 1.1863;
    const float k2 = 2842.5;
    const float k3 = 0.1236;

    cv::Mat1f& depth_im = m_image->depthRef();
    for_all_rc(depth_im)
    {
      float depth = depth_im(r,c);
      if (depth < 2047)
      {
        depth = k3 * tanf(depth/k2 + k1);
      }
      else
        depth = 0;
      depth_im(r,c) = depth;
    }
  }

  void RGBDProcessor :: computeKinectDepthBaseline()
  {
    cv::Mat1f& depth_im = m_image->depthRef();

    double depth_baseline = 7.5e-02;
    double depth_offset = 1090;
    if (m_image->calibration())
    {
      depth_baseline = m_image->calibration()->depth_baseline;
      depth_offset = m_image->calibration()->depth_offset;
    }

    for_all_rc(depth_im)
    {
      float raw_depth = depth_im(r,c);
      float depth = 0;
      if (raw_depth < 2047)
      {
        depth = 540.0 * 8.0 * depth_baseline / (depth_offset - raw_depth);
      }
      if (depth < 0)
        depth = 0;
      else if (depth > 10)
        depth = 10;
      depth_im(r,c) = depth;
    }
  }

  void RGBDProcessor :: computeKinectDepthLinear()
  {
    cv::Mat1f& depth_im = m_image->depthRef();
    for_all_rc(depth_im)
    {
      float raw_depth = depth_im(r,c);
      float depth = 0;
      if (raw_depth < 2047)
      {
        depth = 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
      }
      if (depth < 0)
        depth = 0;
      else if (depth > 10)
        depth = 10;
      depth_im(r,c) = depth;
    }
  }

  void RGBDProcessor :: applyDepthThreshold()
  {
    cv::Mat1b& mask_im = m_image->depthMaskRef();
    const cv::Mat1f& depth_im = m_image->depth();

    for_all_rc(depth_im)
    {
      float depth = depth_im(r,c);
      if (depth < m_min_depth || depth > m_max_depth)
        mask_im(r,c) = 0;
    }
  }

  void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat3b& color_depth_im,
                                   double* i_min_val, double* i_max_val)
  {
    double min_val, max_val;
    if (i_min_val && i_max_val)
    {
      min_val = *i_min_val;
      max_val = *i_max_val;
    }
    else
    {
      minMaxLoc(depth_im, &min_val, &max_val);
    }

    color_depth_im.create(depth_im.size());
    for (int r = 0; r < depth_im.rows; ++r)
    {
      const float* depth_data = depth_im.ptr<float>(r);
      Vec3b* depth_color_data = color_depth_im.ptr<Vec3b>(r);
      for (int c = 0; c < depth_im.cols; ++c)
      {
        int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
        if (v < 0) v = 0;
        char r,g,b;
        int lb = v & 0xff;
        switch (v / 256) {
        case 0:
          r = 255;
          g = 255-lb;
          b = 255-lb;
          break;
        case 1:
          r = 255;
          g = lb;
          b = 0;
          break;
        case 2:
          r = 255-lb;
          g = 255;
          b = 0;
          break;
        case 3:
          r = 0;
          g = 255;
          b = lb;
          break;
        case 4:
          r = 0;
          g = 255-lb;
          b = 255;
          break;
        case 5:
          r = 0;
          g = 0;
          b = 255-lb;
          break;
        default:
          r = 0;
          g = 0;
          b = 0;
          break;
        }
        if (v == 0)
        {
          r = g = b = 0;
        }
        depth_color_data[c] = Vec3b(b,g,r);
      }
    }
  }

} // ntk
