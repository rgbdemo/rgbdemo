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

#include <ntk/utils/debug.h>
#include <ntk/stats/estimation.h>
#include <ntk/stats/distributions.h>
#include <ntk/stats/hypothesis_testing.h>
#include <ntk/stats/moments.h>
#include <ntk/numeric/utils.h>

#include <vnl/vnl_random.h>
#include <QFile>
#include <QTextStream>

using namespace ntk;

void test_mann_whitney_pfa()
{
  {
    std::vector<double> d1(18, 0);
    std::vector<double> d2(18, 0);
    d1[13]=1; d1[12]=2; d1[10]=4; d1[9]=1; d1[8]=2; d1[7]=5; d1[6]=1;
    d2[17]=1; d2[16]=1; d2[15]=3; d2[14]=3; d2[13]=3; d2[12]=4; d2[11]=2; d2[10]=3; d2[8]=2; d2[6]=1;
    double r = compute_mann_whitney_log_pfa(d1, d2); /* u should be 64 */
    assert(ceil(r) == -8.);
  }
  
  {
    std::vector<double> d1(111, 0);
    std::vector<double> d2(111, 0);
    d1[78]=1; d1[64]=1; d1[75]=1; d1[45]=1; d1[82]=1;
    d2[110]=1; d2[70]=1; d2[53]=1; d2[51]=1;
    double r = compute_mann_whitney_log_pfa(d1, d2); /* u should be 9 */
  }
   
  {
    std::vector< std::vector< std::vector<double> > > mw_values = generate_mann_whitney_values();
    int n1 = 4, n2 = 5, u = 9;
  }
} 

int main(int argc, char** argv)
{
  {
    QMap<double,double> distrib;
    distrib[8] = 1;
    distrib[58] = 1;
    distrib[122] = 1;
    distrib[133] = 1;
    distrib[169] = 1;
    ExpDistrib model(98);
    ExpDistrib::Cdf cdf = model.cdf();
    ntk_dbg_print(kolmogorov_sqrt_n_Dn(distrib, cdf), 0);
    Q_ASSERT(flt_eq(cramer_von_mises_nw2(distrib, cdf), 0.0915486, 1e-3));
    Q_ASSERT(flt_eq(kolmogorov_sqrt_n_Dn(distrib, cdf), 0.69772, 1e-3));
  }

  if (argc != 2)
  {
    exit(0);
  }

  std::cout << "Testing plot file." << std::endl;
  QFile plot_file (argv[1]);
  if (!plot_file.open(QIODevice::ReadOnly))
  {
    std::cerr << "Could not open " << argv[1] << std::endl;
    exit(2);
  }

  QMap<double,double> distrib;
  double n = 0;
  QTextStream stream (&plot_file);
  while (!stream.atEnd())
  {
    double x, y;
    stream >> x >> y;
    if (y > 0)
      distrib[x] += y;
    n += y;
  }
  Q_ASSERT(distrib.begin() != distrib.end());

  {
    typedef QMap<double,double> distrib_type;
    std::ofstream f ("data.plot");
    double sum = 0;
    foreach_const_it(it, distrib, distrib_type)
    {
      sum += it.value();
      f << it.key() << " " << sum/n << std::endl;
    }
  }

  // weibull estimate
  if (0)
  {
    WeibullEstimates e;
    e.beta = 1; e.etha = 2; e.gamma = distrib.begin().key()*0.9;
    estimate_weibull3(distrib, e);
    // ntk_dbg_print(e, 0);
    ntk_dbg_print(wblinv(0.99995, e.beta, e.etha, e.gamma), 0);
    WeibullDistrib model (e.beta, e.etha, e.gamma);
    WeibullDistrib::Cdf cdf = model.cdf();
  // Wblcdf cdf(estimates.beta, estimates.etha);
    double nw2 = cramer_von_mises_nw2(distrib, cdf);
    ntk_dbg_print(nw2, 0);
    double dn = kolmogorov_sqrt_n_Dn(distrib, cdf);
    ntk_dbg_print(dn, 0);

    {
      typedef QMap<double,double> distrib_type;
      std::ofstream fweib("weibull.plot");
      foreach_const_it(it, distrib, distrib_type)
        fweib << it.key() << " " << cdf(it.key()) << std::endl;
    }

    WeibullEstimates bounds = estimate_weibull_upper_bounds(distrib, e,
        0.99995, 0.99995);
    // ntk_dbg_print(bounds, 0);
    ntk_dbg_print(wblinv(0.99995, bounds.beta, bounds.etha, bounds.gamma), 0);
  }

  // log normal estimation
  if (1)
  {
    LogNormalDistrib::Params p(0,1);
    double logl = estimate_lognormal(distrib, p);
    ntk_dbg_print(logl, 0);
    LogNormalDistrib m (p);
    LogNormalDistrib::Cdf cdf = m.cdf();
    {
      typedef QMap<double,double> distrib_type;
      std::ofstream flog("lognormal.plot");
      double sum = 0;
      foreach_const_it(it, distrib, distrib_type)
      {
        sum += it.value();
        flog << it.key() << " " << cdf(it.key()) << std::endl;
      }
    }
    double nw2 = cramer_von_mises_nw2(distrib, cdf);
    ntk_dbg_print(nw2, 0);
    double dn = kolmogorov_sqrt_n_Dn(distrib, cdf);
    ntk_dbg_print(dn, 0);
  }

  // fisher tippett estimation
  if (0)
  {
    double mean, dev;
    distrib_moments(distrib, mean, dev);
    FisherTippettDistrib::Params p(mean, dev);
    double logl = estimate_fishertippett(distrib, p);
    ntk_dbg_print(logl, 0);
    FisherTippettDistrib m (p);
    FisherTippettDistrib::Cdf cdf = m.cdf();

    double nw2 = cramer_von_mises_nw2(distrib, cdf);
    ntk_dbg_print(nw2, 0);
    double dn = kolmogorov_sqrt_n_Dn(distrib, cdf);
    ntk_dbg_print(dn, 0);

    {
      typedef QMap<double,double> distrib_type;
      std::ofstream flog("fisher.plot");
      foreach_const_it(it, distrib, distrib_type)
      {
        flog << it.key() << " " << cdf(it.key()) << std::endl;
      }
    }
  }

  // frechet estimation
  {
    double mean, dev;
    distrib_moments(distrib, mean, dev);
    FrechetDistrib::Params p(distrib.begin().key()*0.9, dev, 1);
    double logl = estimate_frechet3(distrib, p);
    ntk_dbg_print(logl, 0);
    FrechetDistrib m (p);
    FrechetDistrib::Cdf cdf = m.cdf();

    double nw2 = cramer_von_mises_nw2(distrib, cdf);
    ntk_dbg_print(nw2, 0);
    double dn = kolmogorov_sqrt_n_Dn(distrib, cdf);
    ntk_dbg_print(dn, 0);

    {
      typedef QMap<double,double> distrib_type;
      std::ofstream flog("frechet.plot");
      foreach_const_it(it, distrib, distrib_type)
      {
        flog << it.key() << " " << cdf(it.key()) << std::endl;
      }
    }
  }
}
