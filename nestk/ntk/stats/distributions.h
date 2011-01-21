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

#ifndef    NTK_STATS_DISTRIBUTIONS_H_
# define    NTK_STATS_DISTRIBUTIONS_H_

# include <ntk/core.h>
# include <ntk/utils/debug.h>
# include <ntk/thread/utils.h>
# include <ntk/numeric/utils.h>
# include <ntk/utils/xml_serializable.h>

# include <memory>
# include <string>
# include <sstream>
# include <fstream>
# include <cfloat>
# include <vector>
# include <map>
# include <numeric>
# include <utility>

# include <algorithm>

# include <QReadWriteLock>

namespace ntk
{


unsigned draw_from_distrib(const std::vector<double>& distrib,
                           cv::RNG& rgen);

template <class T, class U>
double distrib_size(const std::map<T, U>& distrib)
{
  typedef std::map<T, U> distrib_type;
  double nbelem = 0;
  foreach_const_it(it, distrib, typename distrib_type)
  { nbelem += it->second; }
  return nbelem;
}

template <class T>
double distrib_norm(const std::vector<T>& distrib)
{
  return std::accumulate(stl_bounds(distrib), T(0));
}

class Distribution : public ntk::XmlSerializable
{
public:
  virtual double mean() const { ntk_assert(0, "not implemented"); return 0; }
  virtual double deviation() const { ntk_assert(0, "not implemented"); return 0; }

  // P(X = x)
  virtual double pdf(double x) const { ntk_assert(0, "not implemented"); return 0; }
  virtual double logPdf(double x) const { return log(pdf(x)); }

  // P(X <= x)
  virtual double cdf(double x) const { ntk_assert(0, "not implemented"); return 0;}
  virtual double logCdf(double x) const { return log(cdf(x)); }

  // P(X > x)
  virtual double icdf(double x) const { return 1.0 - cdf(x); }
  virtual double ilogCdf(double x) const { return log(icdf(x)); }

  // Value x such that P(X <= x) = t
  virtual double pValue(double x) const { ntk_assert(0, "not implemented."); return 0.; }

  virtual double sample(cv::RNG& rgen) const { ntk_assert(0, "not implemented."); return 0; }

  virtual void estimate(const std::map<double,double>& raw_pdf) { ntk_assert(0, "not implemented."); }

  virtual void saveToPlot(double start, double step, double end, const char* filename)
  {
    std::ofstream f(filename);
    for (double x = start; x < end; x += step)
    {
      f << x << " " << pdf(x) << std::endl;
    }
  }
};

class StudentDistribution : public Distribution
{
public:
  StudentDistribution(double degrees_of_freedom)
    : degree(degrees_of_freedom)
  {}

  StudentDistribution() : degree(1)
  {}

  virtual double pValue(double x) const;

public:
  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "degree", degree); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "degree", degree); }

public:
  double degree;
};

class NormalDistribution : public Distribution
{
public:
  NormalDistribution(double mu, double sigma)
    : mu(mu), sigma(sigma)
  {}

  NormalDistribution() : mu(0), sigma(1)
  {}

  virtual double cdf(double x) const
  {
    double y = (x - mu) / sigma;
    return 0.5 * erfc(-y / sqrt(2.0));
  }

  virtual double logCdf(double x) const
  {
    return lnnormcdf(x, mu, sigma);
  }

  virtual double ilogCdf(double x) const
  {
    return lnnormicdf(x, mu, sigma);
  }

  virtual double icdf(double x) const
  {
    return normcdf(-(x-mu), 0, sigma);
  }

  virtual double pValue(double x) const
  { return (-sqrt(2.0) * erfcinv(2.0*x))*sigma + mu; }

  virtual double pdf(double x) const
  {
    double r = 1. / (sigma * sqrt(2.*M_PI));
    r *= exp(-(x - mu) * (x - mu) / (2. * sigma * sigma));
    return r;
  }

  virtual double logPdf(double x) const
  {
    double a = (x - mu) / sigma;
    double pfa = 0.5 * erfcx(-a / sqrt(2.0));
    pfa = log(pfa) - (a * a / 2.);
    return pfa;
  }

  virtual double sample(cv::RNG& rgen) const
  {
    double x1, x2, w, y1, y2;
    do
    {
      x1 = 2.0f * rgen.uniform(0.0,1.0) - 1.0f;
      x2 = 2.0f * rgen.uniform(0.0,1.0) - 1.0f;
      w = x1 * x1 + x2 * x2;
    }
    while (w >= 1.0f);
    w = sqrt((-2.0 * log(w)) / w);
    y1 = x1 * w * sigma + mu;
    y2 = x2 * w * sigma + mu;
    return y1;
  }


public:
  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "mu", mu); setXmlAttribute(element, "sigma", sigma); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "mu", mu); loadFromXmlAttribute(element, "sigma", sigma); }

public:
  double mu;
  double sigma;
};

class GumbellDistribution : public Distribution
{
public:
  GumbellDistribution(double mu, double beta)
    : mu(mu), beta(beta)
  {}

  GumbellDistribution() : mu(0), beta(1)
  {}

  virtual double pdf(double x) const
  {
    double z = exp(-(x-mu)/beta);
    return (z*exp(-z))/beta;
  }

  virtual double logPdf(double x) const
  {
    double z = exp(-(x-mu)/beta);
    double logz = -(x-mu)/beta;
    return logz - z - log(beta);
  }

  virtual double cdf(double x) const
  {
    return exp(-exp(-(x-mu)/beta));
  }

  virtual double logCdf(double x) const
  {
    return -exp(-(x-mu)/beta);
  }

  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "mu", mu); setXmlAttribute(element, "beta", beta); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "mu", mu); loadFromXmlAttribute(element, "beta", beta); }

public:
  double mu;
  double beta;
};


class GeneralizedParetoDistribution : public Distribution
{
public:
  enum Kind { HeavyTail = 0, NormalTail = 1, ThinTail = 2 };

public:
  GeneralizedParetoDistribution(Kind kind, double scale, double shape,
                                bool has_inf_value = false, double inf_value = 0)
    : m_kind(kind), m_scale(scale), m_shape(shape),
      m_has_inf_value(has_inf_value), m_inf_value(inf_value)
  {
    if (kind == ThinTail)
    {
      ntk_assert(m_has_inf_value, "Inf value required for thin tail.");
      ntk_assert(m_shape < 0, "Shape must be < 0 for thin tail");
      m_scale = -m_shape * m_inf_value;
    }
  }

  void setInfLimit(double inf_value)
  {
    m_has_inf_value = true;
    m_inf_value = inf_value;
  }

  GeneralizedParetoDistribution() :
    m_kind(NormalTail), m_scale(1), m_shape(1),
    m_has_inf_value(false), m_inf_value(0)
  {}

  virtual double pdf(double x) const
  {
    ntk_assert(0, "not implemented");
    return 1;
  }

  virtual double logPdf(double x) const
  {
    switch (m_kind)
    {
    case NormalTail:
      return -log(m_scale) - (x / m_scale);
      break;
    case HeavyTail:
    case ThinTail:
    {
      double r = -log(m_scale);
      r += ((-1.0/m_shape)-1.0) * log(1.0+m_shape*(x/m_scale));
      return r;
      break;
    }
    default:
      ntk_assert(0, "not implemented");
    }
    return 0;
  }

  virtual double cdf(double x) const
  {
    switch (m_kind)
    {
    case NormalTail:
      return exp(-x / m_scale);
      break;

    case HeavyTail:
    case ThinTail:
      return pow(1.0 + m_shape*(x/m_scale), -(1.0/m_shape));
      break;

    default:
      ntk_assert(0, "not implemented");
    }
    return 0;
  }

  virtual double logCdf(double x) const
  {
    switch (m_kind)
    {
    case NormalTail:
      return -x / m_scale;
      break;

    case HeavyTail:
    case ThinTail:
      return -(1.0/m_shape) * log(1.0 + m_shape*(x/m_scale));
      break;

    default:
      ntk_assert(0, "not implemented");
    }
    return 0;
  }

  virtual void estimate(const std::map<double,double>& raw_pdf);

  virtual void fillXmlElement(XMLNode& element) const
  {
    setXmlAttribute(element, "kind", (int)m_kind);
    setXmlAttribute(element, "scale", m_scale);
    setXmlAttribute(element, "shape", m_shape);
  }
  virtual void loadFromXmlElement(const XMLNode& element)
  {
    int k; loadFromXmlAttribute(element, "kind", k); m_kind = (Kind)k;
    loadFromXmlAttribute(element, "scale", m_scale);
    loadFromXmlAttribute(element, "shape", m_shape);
  }

public:
  Kind m_kind;
  double m_scale;
  double m_shape;
  bool m_has_inf_value;
  double m_inf_value;
};

class ExponentialDistribution : public Distribution
{
public:
  ExponentialDistribution(double lambda)
    : lambda(lambda)
  {}

  ExponentialDistribution() : lambda(1)
  {}

  virtual double pdf(double x) const
  {
    return lambda * exp(-lambda*x);
  }

  virtual double logPdf(double x) const
  {
    return log(lambda) - lambda*x;
  }

  virtual double cdf(double x) const
  {
    return 1.0 - exp(-lambda * x);
  }

  virtual double icdf(double x) const
  {
    return exp(-lambda * x);
  }

  virtual double ilogCdf(double x) const
  {
    return -lambda * x;
  }

public:
  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "lambda", lambda); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "lambda", lambda); }

public:
  double lambda;
};


class OneMinusExpDistribution : public Distribution
{
public:
  OneMinusExpDistribution(double lambda) : lambda(lambda)
  {}

  OneMinusExpDistribution (const std::map<double,double>& distrib,
                           double* loglikelihood = 0);

  OneMinusExpDistribution() : lambda(1)
  {}

  virtual void estimate(const std::map<double,double>& raw_pdf)
  { *this = OneMinusExpDistribution(raw_pdf); }

public:
  virtual double logPdf(double x) const
  { x=1-x; if (x < 0) return 0; return log(lambda) - lambda * x; }

  virtual double logCdf(double x) const
  { x=1-x; if (x<0) return 0.0; return -x / lambda; }

public:
  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "lambda", lambda); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "lambda", lambda); }

public:
  double lambda;
};

class NegativeLogLogisticDistribution : public Distribution
{
public:
  NegativeLogLogisticDistribution(double mu, double sigma) : mu(mu), sigma(sigma)
  {}

  NegativeLogLogisticDistribution (const std::map<double,double>& distrib,
                                   double* loglikelihood = 0);

  NegativeLogLogisticDistribution() : mu(0), sigma(1)
  {}

  virtual void estimate(const std::map<double,double>& raw_pdf)
  { *this = NegativeLogLogisticDistribution(raw_pdf); }

public:
  virtual double logPdf(double x) const
  {
    if (x > 0) return -FLT_MAX;
    x = -(log(-x)-mu)/sigma;
    double ex = exp(x);
    return x - log(sigma) - 2.0*log(1+ex);
  }

  virtual double logCdf(double x) const
  {
    if (x > 0) return -FLT_MAX;
    x = -(log(-x)-mu)/sigma;
    if (x > 50) return 0.0;
    return x - log(1 + exp(x));
  }

public:
  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "mu", mu); setXmlAttribute(element, "sigma", sigma); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "mu", mu); loadFromXmlAttribute(element, "sigma", sigma); }

public:
  double mu;
  double sigma;
};

class LogLogisticDistribution : public Distribution
{
public:
  LogLogisticDistribution(double mu, double sigma) : mu(mu), sigma(sigma)
  {}

  LogLogisticDistribution (const std::map<double,double>& distrib,
                           double* loglikelihood = 0);

  LogLogisticDistribution() : mu(0), sigma(1)
  {}

  virtual void estimate(const std::map<double,double>& raw_pdf)
  { *this = LogLogisticDistribution(raw_pdf); }

public:
  virtual double logPdf(double x) const
  {
    if (x < 0) return -FLT_MAX;
    x = -(log(x)-mu)/sigma;
    double ex = exp(x);
    return x - log(sigma) - 2.0*log(1+ex);
  }

  virtual double logCdf(double x) const
  {
    if (x < 0) return -FLT_MAX;
    x = -(log(x)-mu)/sigma;
    if (x > 10) return -x;
    return -log(1 + exp(x));
  }

public:
  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "mu", mu); setXmlAttribute(element, "sigma", sigma); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "mu", mu); loadFromXmlAttribute(element, "sigma", sigma); }

public:
  double mu;
  double sigma;
};

class LogNormalDistribution : public Distribution
{
public:
  LogNormalDistribution(double mu, double sigma) : m_normal(mu, sigma), mu(mu), sigma(sigma)
  {}

  LogNormalDistribution (const std::map<double,double>& distrib,
                         double* loglikelihood = 0);

  LogNormalDistribution() : m_normal(0, 1), mu(0), sigma(1)
  {}

  virtual void estimate(const std::map<double,double>& raw_pdf)
  { *this = LogNormalDistribution(raw_pdf); }

public:
  virtual double logPdf(double x) const
  {
    return m_normal.logPdf(log(x));
  }

  virtual double pdf(double x) const
  {
    return m_normal.pdf(log(x));
  }

  virtual double logCdf(double x) const
  {
    return m_normal.logCdf(log(x));
  }

public:
  virtual void fillXmlElement(XMLNode& element) const
  { setXmlAttribute(element, "mu", mu); setXmlAttribute(element, "sigma", sigma); }
  virtual void loadFromXmlElement(const XMLNode& element)
  { loadFromXmlAttribute(element, "mu", mu); loadFromXmlAttribute(element, "sigma", sigma); }

public:
  NormalDistribution m_normal;
  double mu;
  double sigma;
};

class EmpiricalDistribution;
class ParetoTailExtrapolator
{
public:
  ParetoTailExtrapolator(const EmpiricalDistribution& distrib);

  void estimate(const EmpiricalDistribution& distrib);
  double extrapolateLogCdf(double x) const;
  double extrapolateCdf(double x) const;

private:
  double m_initial_p;
  double m_initial_x;
  GeneralizedParetoDistribution m_pareto;
};
ntk_ptr_typedefs(ParetoTailExtrapolator);

class EmpiricalDistribution : public Distribution
{
  friend class ParetoTailExtrapolator;

public:
  EmpiricalDistribution(double precision = 0.1, bool has_inf_limit = false, double inf_limit = 0)
    : m_has_inf_limit(false), m_inf_limit(inf_limit/precision)
  {
    reset(precision);
  }

public:
  void reset(double precision = 0.1)
  {
    m_precision = precision;
    m_min_value = 0;
    m_max_value = 0,
        m_nb_raw_values = 0;
    m_mean = 0;
    m_variance = 0;
    m_tail_decreasing_rate = 0.99;
  }

public:
  virtual void fillXmlElement(XMLNode& element) const;
  virtual void loadFromXmlElement(const XMLNode& element);

public:
  virtual double mean() const { return m_mean; };
  virtual double deviation() const { return sqrt(m_variance); }
  double minValue() const { return m_min_value * m_precision; }
  double maxValue() const { return m_max_value * m_precision; }
  double precision() const { return m_precision; }
  double tailDecreasingRate() const { return m_tail_decreasing_rate; }

public:
  virtual double cdf(const double x) const;
  virtual double logCdf(const double x) const;

public:
  bool isAccurate() const { QReadLocker locker(&m_lock); return m_nb_raw_values > 10; }
  void closestValueForCdf(double expected_cdf_value, double* closest_x, double* closest_cdf);
  void addValue(double value);
  void addValueFast(double value);
  void finalize();
  void invertValues();

public:
  std::map<double,double> rawPdf() const;
  void setRawPdf(const std::map<double,double>& pdf);
  const std::map<int, double>& rawDistribution() const { return m_raw_distribution; }

public:
  void saveRawPdfAsPlot(const char* f) const;
  void savePdfAsPlot(const char* f) const;
  void saveCdfAsPlot(const char* f) const;
  void saveLogCdfAsPlot(const char* f) const;
  void saveRawLogCdfAsPlot(const char* f) const;
  void saveRawLogICdfAsPlot(const char* f) const;

private:
  double interpolate(double x, const std::pair<double, double>& a, const std::pair<double, double>& b) const;
  double logInterpolate(double x, const std::pair<double, double>& a, const std::pair<double, double>& b) const;
  void computeCdfValues();
  void computeTailDecreasingRate();

private:
  std::map<int, double> m_raw_distribution;
  std::map<int, double> m_cdf_values;
  std::map<int, double> m_icdf_values;
  double m_precision;
  int m_min_value;
  int m_max_value;
  double m_nb_raw_values;
  double m_mean;
  double m_variance;
  double m_tail_decreasing_rate;
  ParetoTailExtrapolatorPtr m_tail_extrapolator;
  bool m_has_inf_limit;
  double m_inf_limit;
  mutable RecursiveQReadWriteLock m_lock;
};
ntk_ptr_typedefs(EmpiricalDistribution);

// Compute the exact law of the mean of n_values iid random variables.
void mean_empirical_distribution(EmpiricalDistribution& mean_distrib,
                                 const std::vector<double>& distrib,
                                 int n_values,
                                 double precision = 1.0);

void mean_empirical_distribution(EmpiricalDistribution& mean_distrib,
                                 const EmpiricalDistribution& raw_distrib,
                                 int n_values);

class EmpiricalMeanDistribution // inherit from Distribution
{
public:
  EmpiricalMeanDistribution(int max_exact_size)
    : m_max_exact_size(max_exact_size), m_raw_distribution(0)
  {}

  void reset(const Distribution& raw_distribution)
  {
    const EmpiricalDistribution* empirical_distrib = dynamic_cast<const EmpiricalDistribution*>(&raw_distribution);
    ntk_assert(empirical_distrib != 0, "Need an empirical distrib !");
    reset(*empirical_distrib);
  }
  void reset(const EmpiricalDistribution& raw_distribution);

  double logCdf(unsigned n, double mean) const;

  const EmpiricalDistribution& rawDistrib(unsigned n) const
  {
    ntk_assert(n > 0 && n <= m_mean_distributions.size(), "No empirical distrib for this size.");
    return *(m_mean_distributions[n-1]);
  }

private:
  int m_max_exact_size;
  const EmpiricalDistribution* m_raw_distribution;
  std::vector<EmpiricalDistributionPtr> m_mean_distributions;
};

} // end of ntk

#endif     /* !NTK_STATS_DISTRIBUTIONS_H_ */
