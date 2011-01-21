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

#include "rgbd_image.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/camera/rgbd_processor.h>

using namespace cv;

namespace ntk
{

  void RGBDImage :: loadFromFile(const std::string& dir,
                                 const RGBDCalibration* calib)
  {
    ntk_assert(0, "not implemented.");
  }

  // Load from a viewXXXX directory.
  void RGBDImage :: loadFromDir(const std::string& dir,
                                const RGBDCalibration* calib,
                                RGBDProcessor* processor)
  {
    m_directory = dir;

    if (!is_file(dir+"/raw/color.png") && is_file(dir+"/color.png"))
    {
      rawRgbRef() = imread(dir + "/color.png", 1);
      rawRgb().copyTo(rgbRef());
      ntk_ensure(rgbRef().data, ("Could not read color image from " + dir).c_str());
    }
    else
    {
      if (is_file(dir + "/raw/color.png"))
      {
        rawRgbRef() = imread(dir + "/raw/color.png", 1);
        ntk_ensure(rawRgbRef().data, ("Could not read raw color image from " + dir).c_str());
      }

      if (is_file(dir + "/raw/depth.yml"))
      {
        rawDepthRef() = imread_yml(dir + "/raw/depth.yml");
        ntk_ensure(rawDepthRef().data, ("Could not read raw depth image from " + dir).c_str());
      }

      if (is_file(dir + "/raw/amplitude.yml"))
      {
        rawAmplitudeRef() = imread_yml(dir + "/raw/amplitude.yml");
        ntk_ensure(rawAmplitudeRef().data, ("Could not read raw amplitude image from " + dir).c_str());
      }
      else if (is_file(dir + "/raw/amplitude.png"))
      {
        rawAmplitudeRef() = imread(dir + "/raw/amplitude.png");
        ntk_ensure(rawAmplitudeRef().data, ("Could not read raw amplitude image from " + dir).c_str());
      }

      if (is_file(dir + "/raw/intensity.png"))
      {
        rawIntensityRef() = imread(dir + "/raw/intensity.png", 0);
        ntk_ensure(rawIntensityRef().data, ("Could not read raw intensity image from " + dir).c_str());
      }
    }

    setCalibration(calib);
    if (processor)
      processor->processImage(*this);
  }

  void RGBDImage :: copyTo(RGBDImage& other) const
  {
    m_rgb.copyTo(other.m_rgb);
    m_rgb_as_gray.copyTo(other.m_rgb_as_gray);
    m_mapped_rgb.copyTo(other.m_mapped_rgb);
    m_depth.copyTo(other.m_depth);
    m_mapped_depth.copyTo(other.m_mapped_depth);
    m_depth_mask.copyTo(other.m_depth_mask);
    m_normal.copyTo(other.m_normal);
    m_amplitude.copyTo(other.m_amplitude);
    m_intensity.copyTo(other.m_intensity);
    m_raw_rgb.copyTo(other.m_raw_rgb);
    m_raw_intensity.copyTo(other.m_raw_intensity);
    m_raw_amplitude.copyTo(other.m_raw_amplitude);
    m_raw_depth.copyTo(other.m_raw_depth);
    other.m_calibration = m_calibration;
    other.m_directory = m_directory;
  }

  void RGBDImage :: swap(RGBDImage& other)
  {
    cv::swap(m_rgb, other.m_rgb);
    cv::swap(m_rgb_as_gray, other.m_rgb_as_gray);
    cv::swap(m_mapped_rgb, other.m_mapped_rgb);
    cv::swap(m_depth, other.m_depth);
    cv::swap(m_mapped_depth, other.m_mapped_depth);
    cv::swap(m_depth_mask, other.m_depth_mask);
    cv::swap(m_normal, other.m_normal);
    cv::swap(m_amplitude, other.m_amplitude);
    cv::swap(m_intensity, other.m_intensity);
    cv::swap(m_raw_rgb, other.m_raw_rgb);
    cv::swap(m_raw_intensity, other.m_raw_intensity);
    cv::swap(m_raw_amplitude, other.m_raw_amplitude);
    cv::swap(m_raw_depth, other.m_raw_depth);
    std::swap(m_calibration, other.m_calibration);
    std::swap(m_directory, other.m_directory);
  }

} // ntk
