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

#include "distributions.h"
#include "estimation.h"
#include "moments.h"

#include <numeric>
#include <algorithm>
#include <ntk/numeric/cost_function.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>

namespace ntk
{

void mean_empirical_distribution(EmpiricalDistribution& mean_distrib,
                                 const std::vector<double>& distrib,
                                 int n_values,
                                 double precision)
{
  mean_distrib.reset();
  cv::Mat_< std::complex<double> > cv_array (1, cv::getOptimalDFTSize(distrib.size()*n_values+1));
  cv_array = std::complex<double>(0);
  foreach_idx(i, distrib) cv_array(0,i) = distrib[i];
  cv::dft(cv_array, cv_array);
  for (int c = 0; c < cv_array.cols; ++c)
    cv_array(0,c) = std::pow(cv_array(0,c), n_values);
  cv::idft(cv_array, cv_array);
  std::map<double,double> raw_pdf;
  for (int i = distrib.size()*n_values; i >=0; --i)
  {
    double p = cv_array(0, i).real()/cv_array.cols;
    if (p > 1e-16)
      raw_pdf[double(i*precision)/n_values] = p;
  }
  mean_distrib.setRawPdf(raw_pdf);
}

void mean_empirical_distribution(EmpiricalDistribution& mean_distrib,
                                 const EmpiricalDistribution& raw_distrib,
                                 int n_values)
{
  mean_distrib.reset(raw_distrib.precision());
  const std::map<int, double>& distrib_map = raw_distrib.rawDistribution();
  if (distrib_map.size() < 2) return;
  int min_value = distrib_map.begin()->first;
  int max_value = distrib_map.rbegin()->first;
  if (min_value < 0)
  {
    ntk_dbg(0) << "Cannot estimate mean for negative values.";
    return;
  }
  std::vector<double> distrib_vector(max_value+1, 0);
  foreach_idx(i, distrib_vector)
  {
    distrib_vector[i] = 10000.0 * (raw_distrib.cdf(double(i) * raw_distrib.precision())
                                   - raw_distrib.cdf(double(i-1.0) * raw_distrib.precision()));
  }

  mean_empirical_distribution(mean_distrib, distrib_vector, n_values, mean_distrib.precision());
}

void EmpiricalMeanDistribution :: reset(const EmpiricalDistribution& raw_distribution)
{
  m_raw_distribution = &raw_distribution;
  m_mean_distributions.clear();
  m_mean_distributions.resize(m_max_exact_size);
  foreach_idx(i, m_mean_distributions)
  {
    EmpiricalDistribution* distrib = new EmpiricalDistribution(m_raw_distribution->precision());
    mean_empirical_distribution(*distrib, *m_raw_distribution, i+1);
    m_mean_distributions[i] = toPtr(distrib);
  }
}

double EmpiricalMeanDistribution :: logCdf(unsigned n, double mean) const
{
  if (n > m_mean_distributions.size())
  {
    return lnnormcdf(mean, m_raw_distribution->mean(), m_raw_distribution->deviation() / sqrt((float)n));
  }

  if (n == 0) return 0;
  return m_mean_distributions[n-1]->logCdf(mean);
}

// Taken from the Gnu Gama library  http://www.gnu.org/software/gama/
double StudentDistribution::pValue(double palfa) const
{
  ntk_assert(palfa >= 0 && palfa <= 1.0, "Bad palfa.");
  palfa = 1.0 - palfa;
  float alfa = palfa;
  double N = degree;
  if (alfa > 0.5) alfa = 1.0 - alfa;
  alfa *= 2;

  if (palfa == 0.5) return 0;

  if (N <= 1)
  {
    float a = M_PI / 2 * alfa;
    float stu_ = cos(a) / sin(a);
    if (palfa > 0.5) stu_ = -stu_;
    return stu_;
  }

  if (N <= 2)
  {
    float stu_ = sqrt(2.0 / (alfa * (2.0 - alfa)) - 2.0);
    if (palfa > 0.5) stu_ = -stu_;
    return stu_;
  }

  float r = N;
  float a = 1.0 / (r - 0.5);
  float b = 48.0 / (a * a);
  float c = ((20700.0 * a / b - 98.0) * a - 16.0) * a + 96.36;
  float d = ((94.5 / (b + c) - 3.0) / b + 1.0) * sqrt(M_PI / 2 * a) * r;
  float x = d * alfa, xx = 2.0 / r;
  float y = pow(x, xx);

  if (y > a + 0.05)
  {
    x = -norminv(0.5 * alfa);
    y = x * x;
    if (N < 5) c += 0.3 * (r - 4.5) * (x + 0.6);
    c = (((0.05 * d * x - 5.0) * x - 7.0) * x - 2.0) * x + b + c;
    y = (((((0.4 * y + 6.3) * y + 36.0) * y + 94.5) / c - y - 3.0) / b + 1.0) * x;
    y = a * y * y;
    if (y <= 0.002)
      y = 0.5 * y * y + y;
    else
      y = exp(y) - 1.0;
    float stu_ = sqrt(r * y);
    if (palfa > 0.5) stu_ = -stu_;
    return stu_;
  }

  y = ((1.0 / (((r + 6.0) / (r * y) - 0.089 * d - 0.822) * (r + 2.0) * 3.0) + 0.5 / (r + 4.0))
       * y - 1.0) * (r + 1.0) / (r + 2.0) + 1.0 / y;
  float stu_ = sqrt(r * y);
  if (palfa > 0.5) stu_ = -stu_;
  return stu_;
}

unsigned draw_from_distrib(const std::vector<double>& distrib, cv::RNG& rgen)
{
  double x = rgen.uniform(0.0,1.0);
  double sum = 0;
  foreach_idx(i, distrib)
  {
    sum += distrib[i];
    if (x <= sum)
      return i;
  }
  return distrib.size() - 1;
}

OneMinusExpDistribution::OneMinusExpDistribution(
  const std::map< double, double > & distrib,
  double* loglikelihood)
{
  typedef std::map<double, double> distrib_type;
  double n = distrib_size(distrib);

  lambda = 0.0;
  foreach_const_it(it, distrib, distrib_type)
      lambda += (1.0 - it->first) * it->second;
  lambda /= n;

  if (loglikelihood) *loglikelihood = model_log_likelihood(distrib, n, *this);
}

NegativeLogLogisticDistribution::NegativeLogLogisticDistribution(
  const std::map< double, double > & distrib,
  double * loglikelihood)
{
  typedef std::map<double, double> distrib_type;
  double n = 0;

  mu = 0.0;
  foreach_const_it(it, distrib, distrib_type)
  {
    if (it->first > -1e-5) continue;
    mu += log(-(it->first)) * it->second;
    n += it->second;
  }
  mu /= n;

  sigma = 0;
  foreach_const_it(it, distrib, distrib_type)
  {
    if (it->first > -1e-5) continue;
    sigma += ntk::math::sqr(log(-(it->first)) - mu) * it->second;
  }
  sigma /= (n - 1);
  sigma = (sigma * 3.0) / (M_PI * M_PI);
  sigma = sqrt(sigma);

  if (loglikelihood) *loglikelihood = model_log_likelihood(distrib, n, *this);
}

LogLogisticDistribution::LogLogisticDistribution(
  const std::map< double, double > & distrib,
  double * loglikelihood)
{
  typedef std::map<double, double> distrib_type;

  double n = 0.0;
  mu = 0.0;
  foreach_const_it(it, distrib, distrib_type)
  {
    if (it->first < 1e-5) continue;
    mu += log((it->first)) * it->second;
    n += it->second;
  }
  mu /= n;

  sigma = 0;
  foreach_const_it(it, distrib, distrib_type)
  {
    if (it->first < 1e-5) continue;
    sigma += ntk::math::sqr(log((it->first)) - mu) * it->second;
  }
  sigma /= (n - 1);
  sigma = (sigma * 3.0) / (M_PI * M_PI);
  sigma = sqrt(sigma);

  if (loglikelihood) *loglikelihood = model_log_likelihood(distrib, n, *this);
}

LogNormalDistribution::LogNormalDistribution(const std::map< double, double > & distrib, double * loglikelihood)
  : m_normal(0,1)
{
  typedef std::map<double, double> distrib_type;

  double n = 0.0;
  mu = 0.0;
  foreach_const_it(it, distrib, distrib_type)
  {
    if (it->first < 1e-5) continue;
    mu += log((it->first)) * it->second;
    n += it->second;
  }
  mu /= n;

  sigma = 0;
  foreach_const_it(it, distrib, distrib_type)
  {
    if (it->first < 1e-5) continue;
    sigma += ntk::math::sqr(log((it->first)) - mu) * it->second;
  }
  sigma /= (n - 1);
  sigma = sqrt(sigma);

  m_normal = NormalDistribution(mu, sigma);

  if (loglikelihood) *loglikelihood = model_log_likelihood(distrib, n, *this);
}

void EmpiricalDistribution :: addValueFast(double value)
{
  QWriteLocker locker(&m_lock);
  int actual_value = ntk::math::rnd(value / m_precision);
  if (m_raw_distribution.find(actual_value) == m_raw_distribution.end())
    m_raw_distribution[actual_value] = 1;
  else
    m_raw_distribution[actual_value] += 1;

  m_mean = recursive_mean(m_mean, m_nb_raw_values, value);
  m_variance = recursive_variance(m_variance, m_mean, m_nb_raw_values, value);

  m_nb_raw_values += 1;

  if (m_nb_raw_values == 1)
  {
    m_min_value = actual_value;
    m_max_value = actual_value;
  }
  else
  {
    m_min_value = std::min(m_min_value, actual_value);
    m_max_value = std::max(m_max_value, actual_value);
  }
}

void EmpiricalDistribution :: finalize()
{
  computeCdfValues();
  computeTailDecreasingRate();
}

void EmpiricalDistribution :: addValue(double value)
{
  QWriteLocker locker(&m_lock);
  int actual_value = ntk::math::rnd(value / m_precision);
  if (m_raw_distribution.find(actual_value) == m_raw_distribution.end())
    m_raw_distribution[actual_value] = 1;
  else
    m_raw_distribution[actual_value] += 1;

  m_mean = recursive_mean(m_mean, m_nb_raw_values, value);
  m_variance = recursive_variance(m_variance, m_mean, m_nb_raw_values, value);

  m_nb_raw_values += 1;

  if (m_nb_raw_values == 1)
  {
    m_min_value = actual_value;
    m_max_value = actual_value;
  }
  else
  {
    m_min_value = std::min(m_min_value, actual_value);
    m_max_value = std::max(m_max_value, actual_value);
  }

  computeCdfValues();
  computeTailDecreasingRate();
}

void EmpiricalDistribution :: setRawPdf(const std::map<double,double>& pdf)
{
  QWriteLocker locker(&m_lock);
  reset(precision());
  bool first = true;
  foreach_const_it(it, pdf, cat2(std::map<double,double>))
  {
    int actual_value = ntk::math::rnd(it->first / m_precision);
    if (first)
    {
      m_min_value = actual_value;
      m_max_value = actual_value;
      first = false;
    }
    else
    {
      m_min_value = std::min(m_min_value, actual_value);
      m_max_value = std::max(m_max_value, actual_value);
    }

    double n_values = it->second*100.0; // make sure values are big enough.

    if (m_raw_distribution.find(actual_value) == m_raw_distribution.end())
      m_raw_distribution[actual_value] = n_values;
    else
      m_raw_distribution[actual_value] += n_values;
    m_mean += (actual_value*m_precision)*n_values;
    m_nb_raw_values += n_values;
  }
  m_mean /= m_nb_raw_values;

  m_variance = 0;
  foreach_const_it(it, m_raw_distribution, cat2(std::map<int,double>))
  {
    double v = it->first*m_precision;
    m_variance += it->second * (m_mean-v) * (m_mean-v);
  }
  m_variance /= m_nb_raw_values;

  computeCdfValues();
  computeTailDecreasingRate();
}

void EmpiricalDistribution :: invertValues()
{
  QWriteLocker locker(&m_lock);
  std::map<int,double> new_raw_values;
  foreach_const_it(it, m_raw_distribution, cat2(std::map<int,double>))
      new_raw_values[-it->first] = it->second;
  std::swap(m_min_value, m_max_value);
  m_min_value *= -1;
  m_max_value *= -1;
  m_mean *= -1;
  m_raw_distribution = new_raw_values;
  computeCdfValues();
}

namespace
{

class linear_regression_function : public ntk::CostFunction
{
public:
  linear_regression_function(const std::map<int, double>& values)
    : CostFunction(2, values.size()),
      m_values(values)
  {}

  virtual void  evaluate (std::vector< double > const &x, std::vector< double > &fx) const
  {
    int i = 0;
    double sum = 0;
    foreach_const_it(it, m_values, cat2(std::map<int,double>))
    {
      fx[i] = ntk::math::sqr(((-x[0]*it->first) + log(it->second) - x[1]) / std::abs(x[0]));
      sum += fx[i];
      ++i;
    }
  }

private:
  const std::map<int,double> m_values;
};

}

#if 0
class GumbellTailExtrapolator : public TailExtrapolator
{
public:
  GumbellTailExtrapolator()
    : m_initial_p(0), m_initial_x(0), m_rate(0)
  {
  }

public:
  void estimate(const std::map<int,double>& values, double initial_x, double initial_p)
  {
    m_initial_x = initial_x;
    m_initial_p = initial_p;

    std::map<int, double>::const_iterator it = values.begin();
    std::map<int, double>::const_iterator next_it = values.end(); --next_it;
    double rate = log(it->second / next_it->second) / std::abs(next_it->first - it->first);

    linear_regression_function f(values);
    vnl_levenberg_marquardt minimizer(f);
    vnl_vector<double> x(2);
    x[0] = -rate;
    it = values.begin();
    x[1] = log(it->second) - x[0] * it->first;
    minimizer.minimize(x);
    ntk_assert(x[0] > 0, "Bad coefficient.");
    m_rate = exp(-x[0]);
  }

  virtual double logExtrapolate(double x) const
  {
    return log(m_rate) * (m_initial_x-x) + log(m_initial_p);
  }

private:
  double m_initial_p;
  double m_initial_x;
  double m_rate;
};

// heavy tail, unbounded
struct pareto_frechet_likelihood_function : public vnl_cost_function
{
  typedef std::map<int,double> distrib_type;

  pareto_likelihood_function(const distrib_type& distrib,
                             ParetoEstimates& e)
    : vnl_cost_function(1), // 1 parameter
      distrib(distrib),
      e(e)
  {
    nbelem = distrib_size(distrib);
  }

  virtual double f (vnl_vector< double > const &x)
  {
    ntk_dbg(3) << "In f";
    e.gamma = x[0];
    if ((distrib.begin()->first-x[0]) <= 0)
      return FLT_MAX;
    estimate_weibull(distrib, e);
    return -e.logl;
  }

private:
  distrib_type distrib;
  double nbelem;
  WeibullEstimates& e;
};
#endif

ParetoTailExtrapolator :: ParetoTailExtrapolator(const EmpiricalDistribution& distrib)
{
  estimate(distrib);
}

void ParetoTailExtrapolator :: estimate(const EmpiricalDistribution& distrib)
{
  const std::map<int,double>& cdf_values = distrib.m_cdf_values;
  const std::map<int,double>& raw_values = distrib.m_raw_distribution;
  m_initial_x = distrib.m_min_value;
  m_initial_p = cdf_values.find(distrib.m_min_value)->second;

  std::map<double, double> filtered_raw_values;
  {
    double n_values = 0;
    int i = 0;
    foreach_const_it(it, raw_values, cat2(std::map<int,double>))
    {
      ++i;
      n_values += distrib.m_raw_distribution.find(it->first)->second;
      if (it->first < m_initial_x)
      {
        filtered_raw_values[(m_initial_x-it->first)] = it->second;
      }
    }
  }

  m_pareto = GeneralizedParetoDistribution();
  ntk_dbg_print(distrib.m_has_inf_limit, 2);
  if (distrib.m_has_inf_limit)
    m_pareto.setInfLimit(m_initial_x-distrib.m_inf_limit);
  m_pareto.estimate(filtered_raw_values);
  // if (distrib.m_has_inf_limit)
}

double ParetoTailExtrapolator :: extrapolateLogCdf(double x) const
{
  double d = m_initial_x - x;
  return m_pareto.logCdf(d) + log(m_initial_p);
}

double ParetoTailExtrapolator :: extrapolateCdf(double x) const
{
  double d = m_initial_x - x;
  return m_pareto.cdf(d) * m_initial_p;
}

void EmpiricalDistribution :: computeTailDecreasingRate()
{
  QWriteLocker locker(&m_lock);

  // Cannot estimate good derivatives.
  if (m_cdf_values.size() < 2)
  {
    return;
  }

#if 0
  if (m_cdf_values.size() < 6)
  {
    std::map<int, double>::const_iterator it = m_cdf_values.begin();
    std::map<int, double>::const_iterator next_it = m_cdf_values.end(); --next_it;
    double rate = log(it->second / next_it->second) / std::abs(next_it->first - it->first);
    m_tail_decreasing_rate = exp(rate);
    return;
  }
#endif

  ParetoTailExtrapolator* extrapolator = new ParetoTailExtrapolator(*this);
  m_tail_extrapolator = toPtr(extrapolator);
}

void EmpiricalDistribution :: computeCdfValues()
{
  QWriteLocker locker(&m_lock);
  m_cdf_values = m_raw_distribution;
  double sum = 0.0;
  double norm = m_nb_raw_values;
  double nb_values = 0;
  foreach_it(it, m_cdf_values, cat2(std::map<int, double>))
  {
    nb_values += it->second;
    if (nb_values < (0.01*m_nb_raw_values))
      m_min_value = it->first;
    sum += (it->second / norm);
    it->second = sum;
  }
  ntk_assert(m_raw_distribution.size() == 0 || flt_eq(sum, 1.0, 1e-5), "Corrupted cdf.");
}

double EmpiricalDistribution ::
interpolate(double x, const std::pair<double, double>& a, const std::pair<double, double>& b) const
{
  ntk_assert(a.first < b.first, "a must be < b");
  return a.second + (x - a.first) * ((b.second - a.second) / (b.first - a.first));
}

double EmpiricalDistribution ::
logInterpolate(double x, const std::pair<double, double>& a, const std::pair<double, double>& b) const
{
  ntk_assert(a.first < b.first, "a must be < b");
  return log(a.second) + (x - a.first) * ((log(b.second) - log(a.second)) / (b.first - a.first));
}

void EmpiricalDistribution ::
closestValueForCdf(double expected_cdf_value, double* closest_x, double* closest_cdf)
{
  QReadLocker locker(&m_lock);
  *closest_x = m_min_value;
  *closest_cdf = 1.0;
  foreach_const_it(it, m_cdf_values, cat2(std::map<int, double>))
  {
    if (it->second > expected_cdf_value)
      break;
    *closest_x = it->first * m_precision;
    *closest_cdf = it->second;
  }
}

double EmpiricalDistribution ::
cdf(const double x) const
{
  QReadLocker locker(&m_lock);
  if (!isAccurate()) return 1.0;
  if (m_min_value == m_max_value) return 1.0;

  // Continuity correction.
  double actual_x = ntk::math::rnd(x / m_precision) + 0.5;

  if (actual_x >= m_max_value)
  {
    return 1.0;
  }

  if (actual_x < m_min_value)
  {
    if (m_tail_extrapolator)
      return m_tail_extrapolator->extrapolateCdf(actual_x);
    return 0;
  }

  std::map<int, double>::const_iterator it, lower_it;
  it = m_cdf_values.upper_bound(actual_x);
  ntk_assert(it != m_cdf_values.end() && it != m_cdf_values.begin(), "Theses cases were tested.");
  lower_it = it; --lower_it; // valid because not begin
  return interpolate(actual_x, *lower_it, *it);
}

double EmpiricalDistribution ::
logCdf(const double x) const
{
  QReadLocker locker(&m_lock);
  if (!isAccurate()) return -1e-10;
  if (m_min_value == m_max_value) return -1e-10;

  // Continuity correction.
  // FIXME: why rnd !!
  double actual_x = (x / m_precision) + 0.5;
  if (actual_x >= m_max_value) return -1e-10;

  if (actual_x < m_min_value)
  {
    // double d = m_min_value - actual_x;
    // double v = log(m_cdf_values.find(m_min_value)->second) + log(m_tail_decreasing_rate) * d;
    double v = -1e-10;
    if (m_tail_extrapolator)
      v = m_tail_extrapolator->extrapolateLogCdf(actual_x);
    return std::min(v, -1e-10);
  }

  std::map<int, double>::const_iterator it, lower_it;
  it = m_cdf_values.upper_bound(actual_x);
  if (it == m_cdf_values.end()) return -1e-10;
  ntk_assert(it != m_cdf_values.end() && it != m_cdf_values.begin(), "Theses cases were tested.");
  lower_it = it; --lower_it; // valid because not begin
  return std::min(1e-10, logInterpolate(actual_x, *lower_it, *it));

  // return std::min(log(cdf(x)), -1e-10);
}

void EmpiricalDistribution ::
fillXmlElement(XMLNode& element) const
{
  const_cast<EmpiricalDistribution*>(this)->computeTailDecreasingRate();

  QReadLocker locker(&m_lock);
  std::vector<double> raw_values;
  foreach_const_it(it, m_raw_distribution, cat2(std::map<int, double>))
  {
    raw_values.push_back(it->first * m_precision);
    raw_values.push_back(it->second);
  }
  setXmlAttribute(element, "raw_values", raw_values);
  setXmlAttribute(element, "nb_raw_values", m_nb_raw_values);

  std::vector<double> cdf_values;
  foreach_const_it(it, m_cdf_values, cat2(std::map<int, double>))
  {
    cdf_values.push_back(it->first * m_precision);
    cdf_values.push_back(it->second);
  }
  setXmlAttribute(element, "cdf_values", cdf_values);

  setXmlAttribute(element, "has-inf-limit", m_has_inf_limit);
  setXmlAttribute(element, "inf-limit", m_inf_limit*m_precision);

  setXmlAttribute(element, "tail-decreasing-rate", m_tail_decreasing_rate);
  setXmlAttribute(element, "mean", m_mean);
  setXmlAttribute(element, "variance", m_variance);
}

void EmpiricalDistribution ::
loadFromXmlElement(const XMLNode& element)
{
  QWriteLocker locker(&m_lock);
  m_raw_distribution.clear();

  std::vector<double> raw_values;
  loadFromXmlAttribute(element, "raw_values", raw_values);
  for (int i = 0; i < (int)raw_values.size(); i += 2)
  {
    m_raw_distribution[ntk::math::rnd(raw_values[i] / m_precision)] = raw_values[i+1];
  }
  loadFromXmlAttribute(element, "nb_raw_values", m_nb_raw_values);

  ntk_dbg_print(m_raw_distribution.size(), 2);
  ntk_dbg_print(m_nb_raw_values, 2);

  m_has_inf_limit = false;
  if (element.getAttribute("has-inf-limit"))
  {
    loadFromXmlAttribute(element, "has-inf-limit", m_has_inf_limit);
    loadFromXmlAttribute(element, "inf-limit", m_inf_limit);
    m_inf_limit /= m_precision;
  }

  loadFromXmlAttribute(element, "mean", m_mean);
  loadFromXmlAttribute(element, "variance", m_variance);
  if (m_raw_distribution.size() > 0)
  {
    m_min_value = m_raw_distribution.begin()->first;
    m_max_value = m_raw_distribution.rbegin()->first;
  }
  else
  {
    m_min_value = 0;
    m_max_value = 0;
  }
  computeCdfValues();
  computeTailDecreasingRate();
}

void EmpiricalDistribution :: savePdfAsPlot(const char* f) const
{
  QReadLocker locker(&m_lock);
  double norm = m_nb_raw_values;

  std::ofstream of(f);
  for (double x = minValue() - (maxValue() - minValue()); x <= maxValue(); x += m_precision)
  {
    of << x << " " << cdf(x + 10.0*m_precision) - cdf(x - 10.0*m_precision) << std::endl;
  }
#if 0
  foreach_const_it(it, m_raw_distribution, cat2(std::map<int, double>))
  {
    of << (it->first*m_precision) << " " << (it->second / norm) << std::endl;
  }
#endif
}

void EmpiricalDistribution :: saveCdfAsPlot(const char* f) const
{
  QReadLocker locker(&m_lock);
  std::ofstream of(f);
  double min_value = minValue() - 4.0 * (maxValue() - minValue());
  if (m_has_inf_limit)
    min_value = std::max(min_value, m_inf_limit);
  for (double x = min_value; x <= maxValue(); x += m_precision)
  {
    of << x << " " << cdf(x) << std::endl;
  }
}

void EmpiricalDistribution :: saveRawPdfAsPlot(const char* f) const
{
  QReadLocker locker(&m_lock);
  std::ofstream of(f);
  foreach_const_it(it, m_raw_distribution, cat2(std::map<int, double>))
  {
    of << it->first*m_precision << " " << it->second << std::endl;
  }
}

void EmpiricalDistribution :: saveRawLogICdfAsPlot(const char* f) const
{
  QReadLocker locker(&m_lock);
  std::ofstream of(f);
  double sum = 0;
  for (std::map<int, double>::const_reverse_iterator it = m_raw_distribution.rbegin();
       it != m_raw_distribution.rend(); ++it)
  {
    sum += it->second / m_nb_raw_values;
    of << it->first*m_precision << " " << log(sum) << std::endl;
  }
}

void EmpiricalDistribution :: saveRawLogCdfAsPlot(const char* f) const
{
  QReadLocker locker(&m_lock);
  std::ofstream of(f);
  foreach_const_it(it, m_cdf_values, cat2(std::map<int, double>))
  {
    of << it->first*m_precision << " " << log(it->second) << std::endl;
  }
}

void EmpiricalDistribution :: saveLogCdfAsPlot(const char* f) const
{
  QReadLocker locker(&m_lock);
  std::ofstream of(f);
  double min_value = minValue() - 8.0 * (maxValue() - minValue());
  if (m_has_inf_limit)
    min_value = std::max(min_value, m_inf_limit);
  for (double x = min_value; x <= maxValue(); x += m_precision)
  {
    of << x << " " << logCdf(x) << std::endl;
  }
}

std::map< double, double > EmpiricalDistribution::rawPdf() const
{
  std::map<double,double> output;
  foreach_const_it(it, m_raw_distribution, cat2(std::map<int,double>))
  {
    output[it->first * m_precision] = it->second;
  }
  return output;
}

struct pareto_normal_tail_likelihood_function : public ntk::CostFunction
{
  typedef std::map<double,double> distrib_type;

  pareto_normal_tail_likelihood_function(const distrib_type& distrib, double scale)
    : CostFunction(1,1), // 1 parameter
      distrib(distrib),
      scale(scale)
  {
    nbelem = distrib_size(distrib);
  }

  virtual void evaluate (const std::vector< double > &x, std::vector<double>& fx) const
  {
    scale = std::max(1e-10, x[0]);
    GeneralizedParetoDistribution model(GeneralizedParetoDistribution::NormalTail, scale, 0);
    double logl = model_log_likelihood(distrib, nbelem, model);
    fx[0] = logl;
  }

private:
  distrib_type distrib;
  double nbelem;
  mutable double scale;
};

struct pareto_heavy_tail_likelihood_function : public ntk::CostFunction
{
  typedef std::map<double,double> distrib_type;

  pareto_heavy_tail_likelihood_function(const distrib_type& distrib)
    : CostFunction(2,1), // 2 parameters
      distrib(distrib)
  {
    nbelem = distrib_size(distrib);
  }

  virtual void evaluate (const std::vector< double > &x, std::vector<double>& fx) const
  {
    if (x[0] < 1e-5 || x[1] < 1e-5)
    {
      fx[0] = FLT_MAX;
      return;
    }
    double scale = x[0];
    double shape = x[1];
    GeneralizedParetoDistribution model(GeneralizedParetoDistribution::HeavyTail, scale, shape);
    double logl = model_log_likelihood(distrib, nbelem, model);
    fx[0] = -logl;
  }

private:
  distrib_type distrib;
  double nbelem;
};

struct pareto_thin_tail_likelihood_function : public ntk::CostFunction
{
  typedef std::map<double,double> distrib_type;

  pareto_thin_tail_likelihood_function(const distrib_type& distrib, double inf_value)
    : CostFunction(1,1), // 1 parameter
      distrib(distrib),
      inf_value(inf_value)
  {
    nbelem = distrib_size(distrib);
  }

  virtual void evaluate (const std::vector< double >&x, std::vector<double>& fx) const
  {
    if (x[0] > -1e-5)
    {
      fx[0] = FLT_MAX;
      return;
    }
    double shape = x[0];
    double scale = -x[0] * inf_value;
    GeneralizedParetoDistribution model(GeneralizedParetoDistribution::ThinTail,
                                        scale, shape, true, inf_value);
    double logl = model_log_likelihood(distrib, nbelem, model);
    fx[0] = -logl;
  }

private:
  distrib_type distrib;
  double nbelem;
  double inf_value;
};

void
GeneralizedParetoDistribution :: estimate(const std::map<double,double>& raw_pdf)
{
  if (m_has_inf_value)
  {
    pareto_thin_tail_likelihood_function f(raw_pdf, m_inf_value);
    LevenbergMarquartMinimizer minimizer;
    std::vector<double> estimate(1);
    estimate[0] = -1;
    minimizer.minimize(f, estimate);
    GeneralizedParetoDistribution model(GeneralizedParetoDistribution::ThinTail,
                                        -1, estimate[0], m_has_inf_value, m_inf_value);
    double thin_tail_logl = model_log_likelihood(raw_pdf,
                                                 distrib_size(raw_pdf),
                                                 model);
    ntk_dbg_print(model.m_scale, 2);
    ntk_dbg_print(model.m_shape, 2);
    ntk_dbg_print(thin_tail_logl, 2);
    *this = model;
    return;
  }

  // No inf value, we have to test the next models.

  GeneralizedParetoDistribution best_model;
  double best_tail_logl = -FLT_MAX;

  double normal_tail_logl = 0;
  {
    pareto_normal_tail_likelihood_function f(raw_pdf, m_scale);
    LevenbergMarquartMinimizer minimizer;
    std::vector<double> estimate(1);
    estimate[0] = m_scale;
    minimizer.minimize(f, estimate);
    GeneralizedParetoDistribution model (GeneralizedParetoDistribution::NormalTail, estimate[0], 0);
    normal_tail_logl = model_log_likelihood(raw_pdf,
                                            distrib_size(raw_pdf),
                                            model);
    if (normal_tail_logl > best_tail_logl)
    {
      best_model = model;
      best_tail_logl = normal_tail_logl;
    }
    ntk_dbg_print(model.m_scale, 2);
    ntk_dbg_print(normal_tail_logl, 2);
  }

  GeneralizedParetoDistribution heavy_model;
  double heavy_tail_logl = 0;
  {
    pareto_heavy_tail_likelihood_function f(raw_pdf);
    LevenbergMarquartMinimizer minimizer;
    std::vector<double> estimate(2);
    estimate[0] = 1;
    estimate[1] = 1;
    minimizer.minimize(f, estimate);
    GeneralizedParetoDistribution model (GeneralizedParetoDistribution::HeavyTail,
                                         estimate[0], estimate[1]);
    heavy_tail_logl = model_log_likelihood(raw_pdf,
                                           distrib_size(raw_pdf),
                                           model);
    if (heavy_tail_logl > normal_tail_logl)
    {
      best_model = model;
      best_tail_logl = heavy_tail_logl;
    }
    ntk_dbg_print(model.m_scale, 2);
    ntk_dbg_print(model.m_shape, 2);
    ntk_dbg_print(heavy_tail_logl, 2);
  }

  *this = best_model;
  ntk_dbg_print(m_kind, 2);
}

}
