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

#ifndef NTK_COLORMODEL_H_
# define NTK_COLORMODEL_H_

# include <ntk/core.h>
# include <opencv/cv.h>

// TODO: use mixture of gaussians?
class HSColorModel
{
public:
  HSColorModel() {}
  ~HSColorModel() { }

public:
  void show() const;
  void build(const cv::Mat3b& model_image, const cv::Mat1b& mask);
  double likelihood(int h_value, int s_value) const;
  void backProject(const cv::Mat3b& bgr_image, cv::Mat1f& likelihood_image) const;

private:
  cv::Mat_<float> m_histogram;
};

#endif // ndef NTK_COLORMODEL_H_
