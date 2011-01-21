/*
** test-estimation.cc
** Login : <nicolas.burrus@ensta.fr>
** Started on  Wed Oct 18 12:51:34 2006 Nicolas Burrus
** $Id$
**
** Copyright (C) 2006, 2007 Nicolas Burrus
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <ntk/ntk.h>
#include <ntk/stats/estimation.h>
#include <ntk/stats/distributions.h>

#include <vnl/vnl_random.h>
#include <QFile>
#include <QTextStream>
#include <map>

#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>

using namespace ntk;

#if 0
void estimate_gamma(std::map<double, double>& distrib)
{
  GammaDistrib gamma_distrib(0.5, 2, 0);
  ntk_dbg_print(gamma_distrib.cdf()(1), 0);
  ntk_dbg_print(gamma_distrib.cdf()(2), 0);
  ntk_dbg_print(gamma_distrib.logPdf()(1), 0);
  ntk_dbg_print(gamma_distrib.logPdf()(2), 0);
  double k = 0.5;
  double theta = 1.0;
  for (int i = 1; i < 1000; i *= 2)
  {
    ntk_dbg_print(i, 0);
    ntk_dbg_print(log(vnl_gamma_q(k, i / theta)), 0);
  }

  GammaDistrib::Params p(0.5, 1, 0);
  double logl = estimate_gamma(distrib, p);
  ntk_dbg_print(logl, 0);
  ntk_dbg_print(p.k, 0);
  ntk_dbg_print(p.theta, 0);
  ntk_dbg_print(p.t, 0);
}
#endif

void test_one_minus_exponential(const std::map<double, double>& distrib)
{
  OneMinusExpDistribution model(distrib);
  ntk_dbg_print(model.lambda, 0);

  std::ofstream f("/tmp/exp.plot");
  foreach_const_it(it, distrib, cat2(std::map<double, double>))
  {
    f << it->first << " " << model.logCdf(it->first) << std::endl;
  }
}

void test_negative_loglogistic(const std::map<double, double>& distrib)
{
  double loglikelihood = 0;
  NegativeLogLogisticDistribution model(distrib, &loglikelihood);
  ntk_dbg_print(model.mu, 0);
  ntk_dbg_print(model.sigma, 0);
  ntk_dbg_print(loglikelihood, 0);

  std::ofstream f("/tmp/logistic.plot");
  foreach_const_it(it, distrib, cat2(std::map<double, double>))
  {
    f << it->first << " " << model.logCdf(it->first) << std::endl;
  }
}

void test_loglogistic(const std::map<double, double>& distrib)
{
  double loglikelihood = 0;
  LogLogisticDistribution model(distrib, &loglikelihood);
  ntk_dbg_print(model.mu, 0);
  ntk_dbg_print(model.sigma, 0);
  ntk_dbg_print(loglikelihood, 0);

  std::ofstream f("/tmp/logistic.plot");
  foreach_const_it(it, distrib, cat2(std::map<double, double>))
  {
    f << it->first << " " << model.logCdf(it->first) << std::endl;
  }
}

void test_lognormal(const std::map<double, double>& distrib)
{
  double loglikelihood = 0;
  LogNormalDistribution model(distrib, &loglikelihood);
  ntk_dbg_print(model.mu, 0);
  ntk_dbg_print(model.sigma, 0);
  ntk_dbg_print(loglikelihood, 0);

  std::ofstream f("/tmp/lognormal.plot");
  foreach_const_it(it, distrib, cat2(std::map<double, double>))
  {
    f << it->first << " " << model.logCdf(it->first) << std::endl;
  }
}

class linear_regression_function : public vnl_least_squares_function
{
  public:
    linear_regression_function(const std::map<int, double>& values)
        : vnl_least_squares_function(2, values.size(), vnl_least_squares_function::no_gradient),
        m_values(values)
    {}

    virtual void  f(vnl_vector< double > const &x, vnl_vector< double > &fx)
    {
      int i = 0;
      double sum = 0;
      foreach_const_it(it, m_values, cat2(std::map<int, double>))
      {
        fx[i] = vnl_math_sqr(((-x[0] * it->first) + log(it->second) - x[1]) / std::abs(x[0]));
        sum += fx[i];
        ++i;
      }
      ntk_dbg_print(sum, 1);
    }

  private:
    const std::map<int, double> m_values;
};

void test_regression()
{
  std::map<int, double> values;
  values[0] = exp(-5); values[1] = exp(-10); values[2] = exp(-15);
  linear_regression_function f(values);
  vnl_levenberg_marquardt minimizer(f);
  vnl_vector<double> x(2);
  x[0] = -1;
  x[1] = 8;
  ntk_dbg_print(x[0], 1);
  ntk_dbg_print(x[1], 1);
  minimizer.minimize(x);  
  ntk_dbg_print(x[0], 1);
  ntk_dbg_print(x[1], 1);
}

int main(int argc, char** argv)
{
  ntk::ntk_debug_level = 1;
  
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " plotfile" << std::endl;
    exit(1);
  }

  QFile plot_file(argv[1]);
  if (!plot_file.open(QIODevice::ReadOnly))
  {
    std::cerr << "Could not open " << argv[1] << std::endl;
    exit(2);
  }

  std::map<double, double> distrib;
  QTextStream stream(&plot_file);
  while (!stream.atEnd())
  {
    double x, y;
    stream >> x >> y;
    if (!stream.atEnd())
      distrib[x] += y;
  }

  if (0)
  {
    test_one_minus_exponential(distrib);
  }
  else if (0)
  {
    test_negative_loglogistic(distrib);
  }
  else if (0)
  {
    test_loglogistic(distrib);
  }
  else if (0)
  {
    test_lognormal(distrib);
  }
  else
  {
    test_regression();
  }
}
