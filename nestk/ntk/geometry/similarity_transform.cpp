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

#include "similarity_transform.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/numeric/utils.h>
#include <ntk/geometry/polygon.h>

using namespace cv;

namespace ntk
{

  void SimilarityTransform :: fillXmlElement(XMLNode& element) const
  {
    addXmlRawTextDataChild(element, "translation", m_translation);
    setXmlAttribute(element, "rotation", m_rotation);
    setXmlAttribute(element, "scale", m_scale);
  }

  void SimilarityTransform :: loadFromXmlElement(const XMLNode& element)
  {
    loadFromXmlRawTextDataChild(element, "translation", m_translation);
    loadFromXmlAttribute(element, "rotation", m_rotation);
    loadFromXmlAttribute(element, "scale", m_scale);
  }

  SimilarityTransform& SimilarityTransform :: applyTransformBefore(const cv::Vec2f& translation,
                                                                   double rotation,
                                                                   double scale)
  {
    SimilarityTransform rhs(Vec2f(0,0), m_rotation, 1);
    Point2f new_translation = rhs.transform(translation);
    m_translation = (m_scale*new_translation) + Point2f(m_translation);
    m_rotation += rotation;
    m_scale *= scale;
    computeMatrix();
    return *this;
  }

  SimilarityTransform& SimilarityTransform :: applyTransformAfter(const cv::Vec2f& translation,
                                                                  double rotation,
                                                                  double scale)
  {
    SimilarityTransform rhs(Vec2f(0,0), rotation, 1);
    Point2f old_translation = rhs.transform(m_translation);
    m_translation = (scale*old_translation) + Point2f(translation);
    m_rotation += rotation;
    m_scale *= scale;
    computeMatrix();
    return *this;
  }

  cv::Point2f SimilarityTransform :: transform(const cv::Point2f& p) const
  {
    Mat1f p_mat (3,1);
    p_mat(0,0) = p.x;
    p_mat(1,0) = p.y;
    p_mat(2,0) = 1;
    cv::Mat1f tmp = m_matrix * p_mat;
    return cv::Point2f(tmp(0,0), tmp(1,0));
  }

  void SimilarityTransform :: transform(const cv::Rect_<float>& rect, Polygon2d& output) const
  {
    output.points.resize(1);

    {
      cv::Point2f p (rect.x, rect.y);
      p = transform(p);
      output.points[0].push_back(p);
    }

    {
      cv::Point2f p (rect.x+rect.width, rect.y);
      p = transform(p);
      output.points[0].push_back(p);
    }

    {
      cv::Point2f p (rect.x+rect.width, rect.y+rect.height);
      p = transform(p);
      output.points[0].push_back(p);
    }

    {
      cv::Point2f p (rect.x, rect.y+rect.height);
      p = transform(p);
      output.points[0].push_back(p);
    }
  }

  void SimilarityTransform :: computeMatrix()
  {
    m_matrix.create(3,3);
    const double cost = cos(deg_to_rad(m_rotation));
    const double sint = sin(deg_to_rad(m_rotation));
    m_matrix(0,0) = m_scale * cost;
    m_matrix(0,1) = -sint * m_scale;
    m_matrix(0,2) = m_translation[0];

    m_matrix(1,0) = sint * m_scale;
    m_matrix(1,1) = m_scale*cost;
    m_matrix(1,2) = m_translation[1];

    m_matrix(2,0) = 0;
    m_matrix(2,1) = 0;
    m_matrix(2,2) = 1;
  }

}
