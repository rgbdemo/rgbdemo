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

#ifndef    NTK_STATS_HISTOGRAM_H_
# define    NTK_STATS_HISTOGRAM_H_

# include <ntk/core.h>
# include <ntk/utils/debug.h>
# include <ntk/utils/xml_serializable.h>

# include <vector>
# include <map>
# include <fstream>
# include <utility>

namespace ntk
{

  template <class Iterator>
  double emd_histogram(Iterator begin1, Iterator end1, Iterator begin2)
  {
    int n = std::distance(begin1, end1);
    std::vector<double> diff_vector(n);
    Iterator it1 = begin1;
    Iterator it2 = begin2;
    int i = 0;
    for (; it1 != end1; ++it1, ++it2, ++i)
    {
      diff_vector[i] = (*it2)-(*it1);
    }
    double min_dist = -1;
    int start_offset = 0;
    for (int k = 0; k < n; ++k)
    {
      double dist = 0;
      double sum = 0;
      int offset = start_offset;
      for (int i = k; i < n; ++i)
      {
        sum += diff_vector[offset];
        dist += std::abs(sum);
        ++offset;
      }
      min_dist = min_dist < 0 ? dist : std::min(dist, min_dist);
      ++start_offset;
    }
    return min_dist;
  }

  template <class Iterator>
  double emd_circular_histogram(Iterator begin1, Iterator end1, Iterator begin2)
  {
    int n = std::distance(begin1, end1);
    std::vector<double> diff_vector(n);
    Iterator it1 = begin1;
    Iterator it2 = begin2;
    int i = 0;
    for (; it1 != end1; ++it1, ++it2, ++i)
    {
      diff_vector[i] = (*it2)-(*it1);
    }
    double min_dist = -1;
    int start_offset = 0;
    for (int k = 0; k < n; ++k)
    {
      double dist = 0;
      double sum = 0;
      int offset = start_offset;
      for (int i = 0; i < n; ++i)
      {
        sum += diff_vector[offset];
        dist += std::abs(sum);
        ++offset;
        if (offset >= n) offset = 0;
      }
      min_dist = min_dist < 0 ? dist : std::min(dist, min_dist);
      ++start_offset;
    }
    return min_dist;
  }

  template <class Iterator>
  double chi2_distance(Iterator begin1, Iterator end1, Iterator begin2)
  {
    double dist = 0;
    while (begin1 != end1)
    {
      double sum = (*begin1 + *begin2);
      double diff = (*begin1 - *begin2);
      if (sum > 1e-10)
      {
        dist += (diff*diff) / sum;
      }
      ++begin1;
      ++begin2;
    }
    return dist;
  }
  
  double emd_distance(const std::vector<double>& h1, const std::vector<double>& h2,
                      int histogram_cycle);

  double cv_emd_distance(const std::vector<double>& h1, const std::vector<double>& h2,
                         int histogram_cycle);

  template <class T>
  double euclidian_distance(const std::vector<T>& d1, const std::vector<T>& d2, double stop_if_greater)
  {
    ntk_assert(d1.size() == d2.size(), "Cannot compute distance.");
    
    typename std::vector<T>::const_iterator desc1, desc2, desc1end;
    desc1 = d1.begin();
    desc1end = d1.end();
    desc2 = d2.begin();

    double dist = 0.0;
    while (desc1 != desc1end && (stop_if_greater < 0 || dist < stop_if_greater))
    {
      double diff = double(*desc1++) - *desc2++;
      dist += diff * diff;
    }
    return dist;
  }
  
  template <class T>
  double absolute_distance(const std::vector<T>& d1, const std::vector<T>& d2)
  {
    ntk_assert(d1.size() == d2.size(), "Cannot compute distance.");
    
    typename std::vector<T>::const_iterator desc1, desc2, desc1end;
    desc1 = d1.begin();
    desc1end = d1.end();
    desc2 = d2.begin();

    double dist = 0.0;
    while (desc1 != desc1end)
    {
      double diff = double(*desc1++) - *desc2++;
      dist += std::abs(diff);
    }
    return dist;
  }
  
  double chi2_distance(const std::vector<double>& d1, const std::vector<double>& d2);

  void get_percentiles_thresholds(const std::vector<double>& histogram,
                                  const double percent,
                                  int& min_value,
                                  int& max_value);
  
} // end of ntk

#endif      /* !NTK_STATS_HISTOGRAM_H_ */
