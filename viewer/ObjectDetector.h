/**
 * This file is part of the rgbdemo project.
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
 * Author: Nicolas Burrus <nicolas@burrus.name>, (C) 2010, 2011
 */

#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <ntk/core.h>

#include <list>

class ObjectDetector
{
public:
  ObjectDetector() : m_min_threshold(1), m_max_threshold(2)
  {}

  void setThresholds(double min, double max)
  { m_min_threshold = min; m_max_threshold = max; }

  double minThreshold() const { return m_min_threshold; }
  double maxThreshold() const { return m_max_threshold; }

  void detect(const cv::Mat1f& distance_image,
              cv::Mat1b& mask_image,
              std::list<cv::Rect>& rects);

private:
  double m_min_threshold;
  double m_max_threshold;
};

#endif // OBJECTDETECTOR_H
