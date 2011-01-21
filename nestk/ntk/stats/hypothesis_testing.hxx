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

# include <algorithm>
# include <map>

namespace ntk
{

  template <class cdf_function>
  double cramer_von_mises_nw2(const std::map<double,double>& distrib, cdf_function f)
  {
    typedef std::map<double,double> distrib_type;
    double n = 0;
    foreach_const_it(it, distrib, distrib_type) n += it->second;
    
    double v = 0;
    double i = 1;
    foreach_const_it(it, distrib, distrib_type)
    {
      double cdf_it = f(it->first);
      for (int j = 0; j < it->second; ++j)
      {
        double t = ((2.0*i-1)/(2.0*n)) - cdf_it;
        v += (t*t);
        i++;
      }
    }
    v += (1.0/(12*n));
    return v;
  }
  
  template <class cdf_function>
  double kolmogorov_sqrt_n_Dn(const std::map<double,double>& distrib, cdf_function f)
  {
    typedef std::map<double,double> distrib_type;
    
    double n = 0;
    foreach_const_it(it, distrib, distrib_type)
      n += it->second;
    
    double sup = 0;
    double cur_cdf = 0;
    foreach_const_it(it, distrib, distrib_type)
    {
      double t1 = std::abs(cur_cdf - f(it->first));
      cur_cdf += (it->second/n);
      double t2 = std::abs(cur_cdf - f(it->first));
      sup = std::max(sup, t1);
      sup = std::max(sup, t2);
    }
    return sqrt(n)*sup;
  }

} // end of ntk

