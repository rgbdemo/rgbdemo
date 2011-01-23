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

#include "ObjectDetector.h"

#include <ntk/ntk.h>

using namespace cv;
using namespace ntk;

void ObjectDetector :: detect(const cv::Mat1f& distance_image,
                              cv::Mat1b& mask_image,
                              std::list<cv::Rect>& rects)
{
  if (mask_image.size() != distance_image.size())
    mask_image = cv::Mat1b(distance_image.size());

  for (int r = 0; r < distance_image.rows; ++r)
    for (int c = 0; c < distance_image.cols; ++c)
    {
      if (distance_image(r,c) >= m_min_threshold && distance_image(r,c) <= m_max_threshold)
        mask_image(r,c) = 255;
      else
        mask_image(r,c) = 0;
    }

  cv::morphologyEx(mask_image, mask_image,
                   cv::MORPH_OPEN,
                   getStructuringElement(cv::MORPH_RECT,
                                         cv::Size(3,3)));
  cv::morphologyEx(mask_image, mask_image,
                   cv::MORPH_CLOSE,
                   getStructuringElement(cv::MORPH_RECT,
                                         cv::Size(3,3)));

  std::vector< std::vector<cv::Point> > contours;
  cv::Mat1b contour_image = mask_image.clone();
  cv::findContours(contour_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  for (int i = 0; i < contours.size(); ++i)
  {
    cv::Rect rect = cv::boundingRect(cv::Mat(contours[i]));
    if (rect.area() > 300)
      rects.push_back(rect);
  }
}
