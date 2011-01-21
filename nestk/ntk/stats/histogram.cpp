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

#include "histogram.h"
#include "moments.h"
#include <ntk/numeric/utils.h>
#include <numeric>
#include <algorithm>

#include <opencv/cv.h>

namespace ntk
{
  
#if 0
  double emd_distance(const std::vector<double>& h1, const std::vector<double>& h2,
                      int histogram_cycle)
  {
    std::vector<double>::const_iterator desc1, desc2, desc1end;
    desc1 = h1.begin();
    desc1end = h1.end();
    desc2 = h2.begin();
    int n = h1.size();
    ntk_assert((n%histogram_cycle)==0, "Not a multiple of cycle !!");

    double dist = 0;
    for (int i = 0; i < n; i += histogram_cycle)
    {
      dist += emd_histogram(desc1+i, desc1+i+histogram_cycle, desc2+i);
    }
    return dist;
  }
#endif

  double cv_emd_distance(const std::vector<double>& h1, const std::vector<double>& h2,
                         int histogram_cycle)
  {
    int n = h1.size();

#if 0
    ntk_dbg_print(h1.size(), 1);
    ntk_dbg_print(h2.size(), 1);

    double norm1 = std::accumulate(stl_bounds(h1), 0.0);
    double norm2 = std::accumulate(stl_bounds(h2), 0.0);
    ntk_dbg_print(norm1, 1);
    ntk_dbg_print(norm2, 1);
#endif

    CvMat* sig1 = cvCreateMat(n, 3, CV_32FC1);
    CvMat* sig2 = cvCreateMat(n, 3, CV_32FC1);

    foreach_idx(i, h1)
    {
      int row = i/histogram_cycle;
      int col = i%histogram_cycle;
      cvSet2D(sig1, i, 0, cvScalar(h1[i]));
      cvSet2D(sig1, i, 1, cvScalar(row));
      cvSet2D(sig1, i, 2, cvScalar(col));

      cvSet2D(sig2, i, 0, cvScalar(h2[i]));
      cvSet2D(sig2, i, 1, cvScalar(row));
      cvSet2D(sig2, i, 2, cvScalar(col));
    }

    double dist = cvCalcEMD2(sig1, sig2, CV_DIST_L2, 0, 0, 0, 0, 0);
    cvReleaseMat(&sig1);
    cvReleaseMat(&sig2);
    return dist;
  }

  double emd_distance(const std::vector<double>& h1, const std::vector<double>& h2,
                      int histogram_cycle)
  {
    int n = h1.size();
    ntk_assert((n%histogram_cycle)==0, "Not a multiple of cycle !!");

    int n_cycle = n/histogram_cycle;

    double global_dist = 0;
    for (int c = 0; c < n_cycle; ++c)
    {
      double cycle_dist = 0;
      double cumul1 = 0;
      double cumul2 = 0;
      for (int i = c*histogram_cycle; i < (c+1)*histogram_cycle; ++i)
      {
        cumul1 += h1[i];
        cumul2 += h2[i];
        cycle_dist += std::abs(cumul1-cumul2);
      }
      global_dist += cycle_dist;
    }
    return global_dist;
  }

  double chi2_distance(const std::vector<double>& d1, const std::vector<double>& d2)
  {
    ntk_assert(d1.size() == d2.size(), "Cannot compare distributions with different sizes !");
    double n1 = std::accumulate(stl_bounds(d1), 0.0);
    double n2 = std::accumulate(stl_bounds(d2), 0.0);
    ntk_assert(n2 > 0.0, "Second histogram cannot be empty.");
    double sum = 0;
    foreach_idx(i, d1)
    {
      if ((d1[i] + d2[i]) > 0)
      {
        double v1 = d1[i];
        double v2 = n1 * d2[i] / n2;
        sum += ntk::math::sqr(v1 - v2) / (v1 + v2);
      }
    }
    return sum;
  }

  void get_percentiles_thresholds(const std::vector< double > & histogram, const double percent, int & min_value, int & max_value)
  {
    double norm = std::accumulate(stl_bounds(histogram), 0.0);
    
    double cumul = 0;
    min_value = 0;
    for (int i = 0; i < (int)histogram.size(); ++i)
    {
      cumul += histogram[i];
      if ((cumul/norm) > percent) break;
      min_value = i;
    }
      
    cumul = 0;
    max_value = histogram.size()-1;
    for (int i = histogram.size()-1; i >= 0; --i)
    {
      cumul += histogram[i];
      if ((cumul/norm) > percent) break;
      max_value = i;
    }
  }
  
} // end of ntk
