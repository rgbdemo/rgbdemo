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

#include "moments.h"
#include <ntk/utils/debug.h>
#include <ntk/numeric/utils.h>

#include <numeric>
#include <limits>
#include <algorithm>
#include <iterator>

namespace ntk
{

  double median(std::vector<double>& values)
  {
    if (values.size() == 1) return values[0];
    std::sort(values.begin(), values.end());
    int n = values.size();
    if (values.size() % 2 == 0)
    {
      return (values[n/2-1] + values[n/2])/2.0;
    }
    return values[n/2];
  }

  std::vector<double>
  fill_linear_values_from_control_points(const std::vector<cv::Point2f>& control_points)
  {
    //std::copy(stl_bounds(control_points), std::ostream_iterator<cv::Point2f>(std::clog, " "));
    std::vector<double> distrib(unsigned(control_points.back().x+1));
    int cp_i = 0;
    foreach_idx(i, distrib)
    {
      while (control_points[cp_i+1].x < i)
      {
        ntk_assert((int)cp_i < (int)control_points.size()-1, "Out of range.");
        cp_i++;
      }
      cv::Point2f left_p = control_points[cp_i];
      cv::Point2f right_p = control_points[cp_i+1];
      double l = (i-left_p.x)/(right_p.x-left_p.x);
      ntk_assert(l <= 1.0, "l is too big.");
      double r = 1.-l;
      double v = left_p.y*r + right_p.y*l;
      distrib[i] = v;
    }
    return distrib;
  }

  void keep_local_maxima(std::vector<cv::Point2f>* maxima, const std::vector<double>& histogram)
  {
    ntk_assert(maxima != 0, "Maxima is null.");
    ntk_assert(histogram.size() >= 2, "Histogram to small.");
    double prev_value = histogram[0];
    maxima->push_back(cv::Point2f(0, histogram[0]));
    for (size_t i = 1; i < histogram.size()-1; ++i)
    {
      double next_value = i < histogram.size()-1 ? histogram[i+1] : -FLT_MAX;
      if (histogram[i] >= prev_value && histogram[i] >= next_value)
        maxima->push_back(cv::Point2f(i, histogram[i]));
      prev_value = histogram[i];
    }
    maxima->push_back(cv::Point2f(histogram.size()-1, histogram.back()));
  }

  std::vector<cv::Point2f> make_unimodal_distribution(const std::vector<cv::Point2f>& points)
  {
    double max_value = -FLT_MAX;
    foreach_idx(i, points)
      if (points[i].y > max_value)
        max_value = points[i].y;

    std::vector<cv::Point2f> control_points;
    double current_max = -FLT_MAX;
    int i = points.size()-1;
    for (i = points.size()-1; i > 0 && points[i].y < max_value; --i)
    {
      if (points[i].y > current_max)
      {
        current_max = points[i].y;
        control_points.insert(control_points.begin(), points[i]);
      }
    }
    int last_index = i;
    control_points.insert(control_points.begin(), points[i]);

    std::vector<cv::Point2f> control_points_fwd;
    current_max = -FLT_MAX;
    for (i = 0; i < last_index; ++i)
    {
      if (points[i].y > current_max)
      {
        current_max = points[i].y;
        control_points_fwd.push_back(points[i]);
      }
    }
    control_points_fwd.insert(control_points_fwd.end(), stl_bounds(control_points));
    return control_points_fwd;
  }

  std::vector<double> distrib_to_bimodal_linear(const std::vector<double>& factors)
  {
    double max_value = -FLT_MAX;
    foreach_idx(i, factors) if (factors[i] > max_value) max_value = factors[i];

    std::vector<cv::Point2f> control_points;
    double current_max = -FLT_MAX;
    int i = factors.size()-1;
    for (i = factors.size()-1; i > 0 && factors[i] < max_value; --i)
    {
      if (factors[i] > current_max)
      {
        current_max = factors[i];
        control_points.insert(control_points.begin(), cv::Point2f(i, factors[i]));
      }
    }
    int last_index = i;
    control_points.insert(control_points.begin(), cv::Point2f(i, factors[i]));

    std::vector<cv::Point2f> control_points_fwd;
    current_max = -FLT_MAX;
    for (i = 0; i < last_index; ++i)
    {
      if (factors[i] > current_max)
      {
        current_max = factors[i];
        control_points_fwd.push_back(cv::Point2f(i, factors[i]));
      }
    }
    control_points_fwd.insert(control_points_fwd.end(), stl_bounds(control_points));
    return fill_linear_values_from_control_points(control_points_fwd);
  }

  double recursive_mean(double previous_mean, double previous_size, double new_value)
  {
    if (previous_size < 0.01) return new_value;

    double t = previous_size + 1;
    return (previous_mean * (t-1) + new_value)/t;
  }

  double recursive_variance(double previous_variance, double new_mean, 
			    double previous_size, double new_value)
  {
    if (previous_size < 0.01) return 0;
    double t = previous_size + 1;
    return (previous_variance*(t-1))/t 
      + ntk::math::sqr(new_value-new_mean)/(t-1);
  }

} // end of ntk

