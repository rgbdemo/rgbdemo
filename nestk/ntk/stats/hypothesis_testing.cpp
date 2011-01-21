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

#include "hypothesis_testing.h"

#include <ntk/numeric/utils.h>
#include <ntk/utils/debug.h>
#include <ntk/stats/distributions.h>

#include <numeric>
#include <cmath>
#include <algorithm>

namespace ntk
{

  double deviation_log_pfa(double dev, double n, double gdev, double gmu4)
  {
    if (n < 4) return 0.;
    if (dev >= gdev) return 0.;

  // Ensures that the estimation is valid.
    if ((gmu4/(gdev*gdev*gdev*gdev) < 1.3)) return 0;

    double vs = ((n-1)/(n*n*n))*((n-1)*gmu4-((n-3)*gdev*gdev*gdev*gdev));
    vs = sqrt(vs);
    double a = dev*dev - ((n-1)/n)*gdev*gdev;
    a /= vs;
  
    double pfa = lnnormcdf(a, 0, 1);
    ntk_assert(pfa < FLT_MAX, "Invalid float values.");
    return pfa;
  }
  
  std::vector< std::vector< std::vector<double> > > generate_mann_whitney_values()
  {
    std::vector< std::vector< std::vector<double> > > result (10);

    for (size_t n2 = 0; n2 < result.size(); ++n2)
    {
      result[n2].resize(result.size());
      for (size_t n1 = 0; n1 < result.size(); ++n1)
      {
        result[n2][n1].resize(result.size()*result.size());
        for (size_t u = 0; u < result.size()*result.size(); ++u)
        {
          if (n1 == 0 || n2 == 0)
          {
            result[n2][n1][u] = 1.;
            continue;
          }
        
          double w1 = ((double)n1)/(n1+n2);
          double t1 = w1 * result[n2][n1-1][u];
          double t2 = (u<n1) ? 0. : (1.-w1) * result[n2-1][n1][u-n1]; 
          result[n2][n1][u] = t1 + t2;          
        }
      }
    }
  
    return result;
  }

  template <class T>
  double compute_mann_whitney_log_pfa(const std::vector<T>& d1,
                                      const std::vector<T>& d2,
                                      double n1, 
                                      double n2)
  {
    static std::vector< std::vector< std::vector<double> > > mw_values 
      = generate_mann_whitney_values();
  
    assert(d1.size() == d2.size());
    if (n1 < 0) n1 = std::accumulate(d1.begin(), d1.end(), 0.);
    if (n2 < 0) n2 = std::accumulate(d2.begin(), d2.end(), 0.);
    double cur_rank = 1;
    double r1 = 0.;
    unsigned size = d1.size();
    for (unsigned i = 0; i < size; ++i)
    {
      const double d1_i = d1[i];
      const double d2_i = d2[i];
      double delta_rank = d1_i + d2_i;
      r1 += 0.5 * ((delta_rank-1) + 2*cur_rank) * d1_i;
      cur_rank += delta_rank;
    }
    double u = n1*n2 + 0.5*n1*(n1+1) - r1;
    u = std::min(u, n1*n2-u);
  
    double r;
  
    if (std::max(n1, n2) < mw_values.size())
      r = log(mw_values[(int)n2][(int)n1][(int)u]);
    else
      r = lnnormcdf(u, 0.5*n1*n2, sqrt((n1*n2*(n1+n2+1))/12.));
    return r;
  }

  template double compute_mann_whitney_log_pfa<double>(const std::vector<double>& d1, 
                                                       const std::vector<double>& d2,
                                                       double n1, double n2);

  template double compute_mann_whitney_log_pfa<int>(const std::vector<int>& d1, 
                                                    const std::vector<int>& d2,
                                                    double n1, double n2);

  template double compute_mann_whitney_log_pfa<unsigned>(const std::vector<unsigned>& d1, 
                                                         const std::vector<unsigned>& d2,
                                                         double n1, double n2);
  
  void estimate_abs_difference_distribution(std::vector<double>& output, 
                                            const std::vector<double>& d1, 
                                            const std::vector<double>& d2)
  {
    ntk_assert(d1.size() == d2.size(), "Distributions must be equally sized");
    output.resize(d1.size());
    std::fill(stl_bounds(output), 0);
    
    double n1 = distrib_norm(d1);
    double n2 = distrib_norm(d2);
    
    for (int d = 0; d < (int)output.size(); ++d)
    {
      for (int i = 0; i < (int)d1.size() - d; ++i)
      {
        int j = (i+d);
        output[d] += (d1[i]/n1)*(d2[j]/n2);
        if (i != j) output[d] += (d1[j]/n1)*(d2[i]/n2);
      }
    }

    // Post condition.
    double norm = distrib_norm(output);
    ntk_assert(flt_eq(norm, 1.0, 1e-3), "Norm is not 1.");
  }
  
} // end of ntk
