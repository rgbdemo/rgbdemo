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

#include "rgbd_frame_recorder.h"

#include <ntk/ntk.h>

#include <QDir>

#include <sys/stat.h>
#include <sys/types.h>

using namespace cv;

namespace ntk
{

  RGBDFrameRecorder :: RGBDFrameRecorder(const std::string& directory)
    : m_frame_index(0), m_only_raw(true)
  {
    setDirectory(directory);
  }

  void RGBDFrameRecorder :: setDirectory(const std::string& directory)
  {
    m_dir = QDir(directory.c_str());
    m_frame_index = 0;
  }

  void RGBDFrameRecorder :: saveCurrentFrame(const RGBDImage& image)
  {
    std::string frame_dir = format("%s/view%04d", m_dir.absolutePath().toStdString().c_str(), m_frame_index);
    std::string raw_frame_dir = format("%s/raw", frame_dir.c_str(), m_frame_index);

    QDir dir (frame_dir.c_str());
    dir.mkpath("raw");

    std::string filename;

    if (!m_only_raw)
    {
      filename = cv::format("%s/color.png", frame_dir.c_str());
      imwrite(filename, image.rgb());
    }

    filename = cv::format("%s/raw/color.png", frame_dir.c_str());
    imwrite(filename, image.rawRgb());

    if (!m_only_raw && image.mappedDepth().data)
    {
      filename = cv::format("%s/mapped_depth.png", frame_dir.c_str());
      imwrite_normalized(filename, image.mappedDepth());

      filename = cv::format("%s/mapped_color.png", frame_dir.c_str());
      imwrite(filename, image.mappedRgb());

      filename = cv::format("%s/depth.yml", frame_dir.c_str());
      imwrite_yml(filename, image.mappedDepth());
    }

    if (!m_only_raw)
    {
      filename = cv::format("%s/raw/depth.png", frame_dir.c_str());
      if (image.rawDepth().data)
        imwrite_normalized(filename.c_str(), image.rawDepth());

      filename = cv::format("%s/depth.png", frame_dir.c_str());
      if (image.depth().data)
        imwrite_normalized(filename.c_str(), image.depth());

      filename = cv::format("%s/intensity.png", frame_dir.c_str());
      if (image.intensity().data)
        imwrite_normalized(filename.c_str(), image.intensity());
    }

    filename = cv::format("%s/raw/depth.yml", frame_dir.c_str());
    if (image.rawDepth().data)
      imwrite_yml(filename.c_str(), image.rawDepth());

    filename = cv::format("%s/raw/intensity.png", frame_dir.c_str());
    if (image.intensity().data)
      imwrite_normalized(filename.c_str(), image.rawIntensity());

    if (!m_only_raw)
    {
      filename = cv::format("%s/raw/amplitude.png", frame_dir.c_str());
      if (image.rawAmplitude().data)
        imwrite_normalized(filename.c_str(), image.rawAmplitude());

      filename = cv::format("%s/amplitude.png", frame_dir.c_str());
      if (image.amplitude().data)
        imwrite_normalized(filename.c_str(), image.amplitude());
    }

    filename = cv::format("%s/raw/amplitude.yml", frame_dir.c_str());
    if (image.rawAmplitude().data)
      imwrite_yml(filename.c_str(), image.rawAmplitude());

    ++m_frame_index;
  }

}
