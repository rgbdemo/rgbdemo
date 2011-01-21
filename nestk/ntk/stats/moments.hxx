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

namespace ntk
{

  template <class T>
  void values_moments(const T& begin, const T& end, unsigned np, double* mean, double* dev)
  {
    T it = begin;
    *mean = 0;
    for (T it = begin; it != end; ++it)
      *mean += *it;
    *mean /= np;
    
    if (!dev) return;

    *dev = 0;
    for (T it = begin; it != end; ++it)      
      *dev += (*it-*mean)*(*it-*mean);
    *dev /= np;
    *dev = sqrt(*dev);
  }
  
  template <class T>
  void distrib_moments(const T& distrib, double& mean, double& dev)
  {
    double np = 0.;
    mean = 0.;
    for (unsigned i = 0; i < distrib.size(); ++i)
    {
      mean += distrib[i]*i;
      np += distrib[i];
    }
    mean /= np;
    dev = 0.;
    for (unsigned i = 0; i < distrib.size(); ++i)
      dev += distrib[i] * (mean-i) * (mean-i);
    dev /= np;
    dev = sqrt(dev);
  }

  template <class T>
  void distrib_moments(const T& distrib, 
                       double& mean, double& dev, double& mu4)
  {
    distrib_moments(distrib, mean, dev);
    
    mu4 = 0.;
    double np = 0.;
    for (unsigned i = 0; i < distrib.size(); ++i)
    {
      double delta = (i-mean)*(i-mean)*(i-mean)*(i-mean);
      mu4 += distrib[i]*delta;
      np += distrib[i];
    }
    mu4 /= np;
  }

} // end of ntk
