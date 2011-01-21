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

#include "utils.h"
#include <ntk/utils/debug.h>

#ifdef USE_GSL
  #include <gsl/gsl_cdf.h>
#endif // def USE_GSL

namespace ntk
{

  long greater_power_of_two(double value)
  {
    long result = 1;
    while (result < value) result *= 2;
    return result;
  }

  double normalize_unoriented_angle_0_pi(double angle)
  {
    angle = normalize_angle_0_2pi(angle);
    if (angle > M_PI) angle -= M_PI;
    return angle;
  }

  double normalize_angle_0_2pi(double angle)
  {
    angle = fmod(angle, 2.0*M_PI);
    if (angle < 0) angle += 2.0*M_PI;
    ntk_assert(ntk::in_range(0.0, angle, 2.0*M_PI), "Angle not in [0, 2pi]");
    return angle;
  }

  double erfcx(double x)
  {
    if (x > 3.) return (ntk::math::two_over_sqrtpi/2.)*(1./x);
    return exp(x*x)*erfc(x);
  }

  double normcdf(double x, double mean, double std_dev)
  {
    double y = (x-mean)/std_dev;
  return 0.5 * erfc(-y/ntk::math::sqrt2);
  }

  double normpdf(double x, double mean, double dev)
  {
    double r = 1. / (dev * sqrt(2.*M_PI));
    r *= exp(-(x-mean)*(x-mean) / (2. * dev * dev));
    return r;
  }

  double lnnormcdf(double x, double mean, double std_dev)
  {
    if (x > mean) return log(normcdf(x, mean, std_dev));
    double a = (x-mean)/std_dev;
    double pfa = 0.5*erfcx(-a/ntk::math::sqrt2);
    pfa = log(pfa)-(a*a/2.);
    return pfa;
  }

  double lnnormicdf(double x, double mean, double std_dev)
  {
    return lnnormcdf(-(x-mean), 0, std_dev);
  }

  double lnchoosek(unsigned n, unsigned k)
  {
    if (k > n) return 0.;
    double res = 0.;
    for (unsigned i = 1; i <= k; ++i)
      res += log(double(n-i+1)) - log(double(i));
    return res;
  }

  /*
   * Comes from codecogs : http://www.codecogs.com/ .
   */
  double erfinv(double y)
  {
    double s, t, u, w, x, z;
    double k = y; // store y before switching its sign
    if (y == 0)
    {
      x = 0;
      goto _end;
    }

    if (y > 1.0)
    {
      x = -log(0.0); // to generate +inf
      goto _end;
    }

    if (y < -1.0)
    {
      x = log(0.0); // to generate -inf
      goto _end;
    }

    // switch the sign of y it it's negative hence the
    // comupation is done as if y >0
    if (y < 0)
      y = -y;

    z = 1.0 - y;
    w = 0.916461398268964 - log(z);
    u = sqrt(w);
    s = (log(u) + 0.488826640273108) / w;
    t = 1 / (u + 0.231729200323405);
    x = u * (1.0 - s * (s * 0.124610454613712 + 0.5)) -
        ((((-0.0728846765585675 * t + 0.269999308670029) * t +
        0.150689047360223) * t + 0.116065025341614) * t +
        0.499999303439796) * t;
    t = 3.97886080735226 / (x + 3.97886080735226);
    u = t - 0.5;
    s = (((((((((0.00112648096188977922 * u +
        1.05739299623423047e-4) * u - 0.00351287146129100025) * u -
        7.71708358954120939e-4) * u + 0.00685649426074558612) * u +
        0.00339721910367775861) * u - 0.011274916933250487) * u -
        0.0118598117047771104) * u + 0.0142961988697898018) * u +
        0.0346494207789099922) * u + 0.00220995927012179067;
    s = ((((((((((((s * u - 0.0743424357241784861) * u -
        0.105872177941595488) * u + 0.0147297938331485121) * u +
        0.316847638520135944) * u + 0.713657635868730364) * u +
        1.05375024970847138) * u + 1.21448730779995237) * u +
        1.16374581931560831) * u + 0.956464974744799006) * u +
        0.686265948274097816) * u + 0.434397492331430115) * u +
        0.244044510593190935) * t -
        z * exp(x * x - 0.120782237635245222);
    x += s * (x * s + 1.0);

    // this line uses the fact that the fuction is symmetric about the origin
    if(k < 0)
      return -x;
    else
      _end: return x;
  }

  double binomial_logicdf_hoeffding_estimate(unsigned k, double p, unsigned l)
  {
    double r = double(k)/l;
    return l*r*log(p/r) + l*(1.0-r)*log((1.0-p)/(1.0-r));
  }

#ifdef USE_GSL

  double binomial_logicdf_gsl(unsigned k, double p, unsigned l)
  {
    return log(gsl_cdf_binomial_Q(k, p, l));
  }

  double binomial_logcdf_gsl(unsigned k, double p, unsigned l)
  {
    return log(gsl_cdf_binomial_P(k, p, l));
  }

  // log(B(X \geq k), l, p) with B a binomial distribution
  double binomial_logicdf_best_estimate(unsigned k, double p, unsigned l)
  {
    if (p < 0.2 && double(k)/l > p)
      return binomial_logicdf_hoeffding_estimate(k,p,l);
    else
      return binomial_logicdf_gsl(k,p,l);
  }

#endif // def USE_GSL

} // end of ntk
