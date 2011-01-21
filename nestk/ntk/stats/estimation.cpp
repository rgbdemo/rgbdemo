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

#include <ntk/core.h>
#include <ntk/stats/estimation.h>
#include <ntk/numeric/utils.h>
#include <ntk/numeric/cost_function.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/stats/distributions.h>
#include <ntk/utils/debug.h>
#include <ntk/utils/stl.h>

#include <iostream>
#include <cfloat>
#include <algorithm>

namespace ntk
{

  double model_log_likelihood(const std::map<double,double>& distrib, 
                              double nbelem,
                              const Distribution& model)
  {
    typedef std::map<double,double> distrib_type;
    double logl = 0;
    foreach_const_it(it, distrib, distrib_type)
    {
      double t = model.logPdf(it->first);
      logl += t*it->second;
    }
    return logl;
  }
  
#if 0
  
  struct weibull_likelihood_function : public vnl_cost_function
  {
    typedef std::map<double,double> distrib_type;
    
    weibull_likelihood_function(const distrib_type& distrib, double gamma)
    	: vnl_cost_function(2), // 2 parameters
        distrib(distrib),
        gamma(gamma)
    {
      nbelem = distrib_size(distrib);
    }
    
    virtual double f (vnl_vector< double > const &x)
    {
      double beta = std::max(0.0, x[0]);
      double etha = std::max(0.0, x[1]);
      WeibullEstimates e; e.beta = beta; e.etha = etha; e.gamma = gamma;
      WeibullDistrib model (beta, etha, gamma);
      e.logl = model_log_likelihood(distrib, nbelem, model);
      // e.logl = weibull_log_likelihood(distrib, nbelem, e);
      return -e.logl;
    }
    
  private:
    distrib_type distrib;
    double nbelem;
    double gamma;
  };
  
  void estimate_weibull(std::map<double,double> distribution, 
                        WeibullEstimates& e)
  {	
    if (has_key(distribution, 0))
      distribution[0.0001] += distribution[0];
    distribution.erase(0);
  	weibull_likelihood_function f(distribution, e.gamma);
    // vnl_lbfgs minimizer(f);
    // vnl_powell minimizer(&f);
    vnl_amoeba minimizer(f);
    vnl_vector<double> estimate(2);
    estimate[0] = e.beta;
    estimate[1] = e.etha;
    // Q_ASSERT(minimizer.minimize(estimate));
    minimizer.minimize(estimate);
    WeibullEstimates estimates;
    e.beta = estimate[0];
    e.etha = estimate[1];
    WeibullDistrib m (e.beta, e.etha, e.gamma);
    e.logl = model_log_likelihood(distribution, 
                                  distrib_size(distribution),
                                  m);
  }
 
  struct weibull3_likelihood_function : public vnl_cost_function
  {
    typedef std::map<double,double> distrib_type;
    
    weibull3_likelihood_function(const distrib_type& distrib, 
                                 WeibullEstimates& e)
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

  void estimate_weibull3(std::map<double,double> distribution, 
                         WeibullEstimates& e)
  {
    if (has_key(distribution, 0))
      distribution[0.0001] += distribution[0];
    distribution.erase(0);
    weibull3_likelihood_function f(distribution, e);
    // vnl_lbfgs minimizer(f);
    // vnl_powell minimizer(&f);
    vnl_amoeba minimizer(f);
    vnl_vector<double> estimate(1);
    estimate[0] = e.gamma;
    minimizer.minimize(estimate);
    ntk_assert(flt_eq(e.gamma, estimate[0]), "");
  }
  
  struct weibull_upper_bounds_function : public vnl_cost_function
  {
		typedef std::map<double,double> distrib_type;
    
    weibull_upper_bounds_function(const distrib_type& distrib,
                                  WeibullEstimates estimates,
                                  double confidence,
                                  double c)
    	: vnl_cost_function(2), // 2 parameters
        distrib(distrib),
        estimates(estimates),
        confidence(confidence),
        c(c),
        gamma(estimates.gamma)
    {
      nbelem = 0;
      foreach_const_it(it, distrib, distrib_type)
        nbelem += it->second;
      WeibullDistrib m (estimates.beta, estimates.etha, estimates.gamma);
      logl_estimates = model_log_likelihood(distrib, nbelem, m);
      ntk_assert(confidence <= 0.99996 && confidence >= 0.99994, "");
    }
    
    virtual double f (vnl_vector< double > const &x)
    {
      double beta = x[0];
      double etha = x[1];
      if (beta <= 0 || etha <= 0)
        return FLT_MAX;
      WeibullEstimates e; e.beta = beta; e.etha = etha; e.gamma = gamma;
      WeibullDistrib m (beta, etha, gamma);
      double logl = model_log_likelihood(distrib, nbelem, m);
      double chi2bound = 19.8070; // FIXME: implement chi2inv(0.9995, 2)
      double t1 = (2.0*(logl-logl_estimates)+chi2bound);
      if (t1 > 0) t1 = 0;
      else if (t1 > -1) t1 = exp(-500*t1);
      else t1 = exp(-1.0*-500.0)-(t1+1)*1e10;
      double t2 = -log(wblinv(c, beta, etha));
      return t1+t2;
    }
    
    virtual void gradf (vnl_vector< double > const &x, 
                        vnl_vector< double > &gradient)
    { fdgradf(x, gradient); }
    
  private:
    distrib_type distrib;
    double nbelem;
    WeibullEstimates estimates;
    double logl_estimates;
    double confidence;
    double c;
    double gamma;
  };
  
  
  // Finds parameters b, n in the confidence region induced by confidence
  // which maximize wblinv(c, b, n)
	WeibullEstimates 
  estimate_weibull_upper_bounds(std::map<double,double> distribution,
                                const WeibullEstimates& best_params,
                                double confidence,
                                double c)
  {
    if (has_key(distribution, 0))
      distribution[0.0001] += distribution[0];
    distribution.erase(0);
	  if (distribution.size() < 5)
      return best_params;
  	weibull_upper_bounds_function f(distribution, best_params, confidence, c);
    // vnl_lbfgs minimizer(f);
    vnl_powell minimizer(&f);
    // vnl_conjugate_gradient minimizer(f);
    vnl_vector<double> estimate(2);
    estimate[0] = best_params.beta;
    estimate[1] = best_params.etha;
    ntk_assert(minimizer.minimize(estimate), "");
    if (std::abs(estimate[0]-best_params.beta) > (best_params.beta*0.8)
        || std::abs(estimate[1]-best_params.etha) > (best_params.etha*0.8))
    {
      std::cerr << "Warning: algorithm did not converge." << std::endl;
      return best_params;
    }
    WeibullEstimates estimates;
    estimates.beta = estimate[0];
    estimates.etha = estimate[1];
    estimates.gamma = best_params.gamma;
    estimates.logl = 0;
    return estimates;
  }
  
  double estimate_lognormal(const std::map<double,double>& distrib,
                            LogNormalDistrib::Params& init)
  {
    typedef std::map<double,double> distrib_type;    
    double n = distrib_size(distrib);
    
    double mu = 0.0;
    foreach_const_it(it, distrib, distrib_type)
      mu += log(it->first)*it->second;
    mu /= n;
    
    double dev = 0;
    foreach_const_it(it, distrib, distrib_type)
    {
      double t = log(it->first)-mu;
      dev += t*t*it->second;
    }
    dev /= n;
    dev = sqrt(dev);
    init.mu = mu;
    init.sigma = dev;
    LogNormalDistrib m(mu, dev);
    return model_log_likelihood(distrib, n, m);
  }
  
  double estimate_exponential(const std::map<double,double>& distrib,
                              ExpDistrib::Params& init)
  {
    typedef std::map<double,double> distrib_type;
    double n = distrib_size(distrib);
    
    double l = 0.0;
    foreach_const_it(it, distrib, distrib_type)
      l += it->first*it->second;
    l /= n;
    
    init.lambda = l;
    ExpDistrib m(l);
    return model_log_likelihood(distrib, n, m);
  }    
  
  struct fishertippett_likelihood_function : public vnl_cost_function
  {
    typedef std::map<double,double> distrib_type;
    
    fishertippett_likelihood_function(const distrib_type& distrib)
      : vnl_cost_function(2), // 2 parameters
        distrib(distrib)
    {
      nbelem = distrib_size(distrib);
    }
    
    virtual double f (vnl_vector< double > const &x)
    {
      double mu = x[0];
      double beta = x[1];
      FisherTippettDistrib model (mu, beta);
      double logl = model_log_likelihood(distrib, nbelem, model);
      return -logl;
    }
    
  private:
    distrib_type distrib;
    double nbelem;
  };
  
  double estimate_fishertippett(const std::map<double,double>& distrib,
                                FisherTippettDistrib::Params& init)
  {
    fishertippett_likelihood_function f(distrib);
    // vnl_lbfgs minimizer(f);
    // vnl_powell minimizer(&f);
    vnl_amoeba minimizer(f);
    vnl_vector<double> estimate(2);
    estimate[0] = init.mu;
    estimate[1] = init.beta;
    // Q_ASSERT(minimizer.minimize(estimate));
    minimizer.minimize(estimate);
    init.mu = estimate[0];
    init.beta = estimate[1];
    FisherTippettDistrib m (init);
    double logl = model_log_likelihood(distrib, 
                                       distrib_size(distrib),
                                       m);
    return logl;
  }
  
  struct frechet_likelihood_function : public vnl_cost_function
  {
    typedef std::map<double,double> distrib_type;
    
    frechet_likelihood_function(const distrib_type& distrib, double mu)
      : vnl_cost_function(2), // 3 parameters
        distrib(distrib),
        mu(mu)
    {
      nbelem = distrib_size(distrib);
    }
    
    virtual double f (vnl_vector< double > const &x)
    {
      double sigma = x[0];
      double alpha = x[1];
      // if (alpha <= 0) return FLT_MAX;
      // if (sigma <= 0) return FLT_MAX;
      FrechetDistrib model (mu, sigma, alpha);
      double logl = model_log_likelihood(distrib, nbelem, model);
      return -logl;
    }
    
  private:
    const distrib_type& distrib;
    double nbelem;
    double mu;
  };
  
  double estimate_frechet(const std::map<double,double>& distrib,
                          FrechetDistrib::Params& init)
  {
    frechet_likelihood_function f(distrib, init.mu);
    // vnl_lbfgs minimizer(f);
    // vnl_powell minimizer(&f);
    vnl_amoeba minimizer(f);
    vnl_vector<double> estimate(2);
    estimate[0] = init.sigma;
    estimate[1] = init.alpha;
    // Q_ASSERT(minimizer.minimize(estimate));
    minimizer.minimize(estimate);
    init.sigma = estimate[0];
    init.alpha = estimate[1];
    FrechetDistrib m (init);
    double logl = model_log_likelihood(distrib, 
                                       distrib_size(distrib),
                                       m);
    return logl;
  }
  
  struct frechet3_likelihood_function : public vnl_cost_function
  {
    typedef std::map<double,double> distrib_type;
    
    frechet3_likelihood_function(const distrib_type& distrib, 
                                 FrechetDistrib::Params init)
      : vnl_cost_function(1), // 1 parameter
        distrib(distrib),
        init(init)
    {
      nbelem = distrib_size(distrib);
    }
    
    virtual double f (vnl_vector< double > const &x)
    {
      init.mu = x[0];
      if ((distrib.begin()->first-init.mu) < 0)
        return FLT_MAX;
      FrechetDistrib::Params p (init);
      double logl = estimate_frechet(distrib, p);
      return -logl;
    }
    
  private:
    distrib_type distrib;
    double nbelem;
    FrechetDistrib::Params init;
  };
  
  double estimate_frechet3(const std::map<double,double>& distrib,
			   FrechetDistrib::Params& init)
  {
    frechet3_likelihood_function f(distrib, init);
    // vnl_lbfgs minimizer(f);
    // vnl_powell minimizer(&f);
    vnl_amoeba minimizer(f);
    vnl_vector<double> estimate(1);
    estimate[0] = init.mu;
    // Q_ASSERT(minimizer.minimize(estimate));
    minimizer.minimize(estimate);
    init.mu = estimate[0];
    estimate_frechet(distrib, init);
    FrechetDistrib m (init);
    double logl = model_log_likelihood(distrib, 
                                       distrib_size(distrib),
                                       m);
    return logl;
  }

  struct gamma_likelihood_function : public vnl_cost_function
  {
    typedef std::map<double,double> distrib_type;
    
    gamma_likelihood_function(const distrib_type& distrib, GammaDistrib::Params& init)
      : vnl_cost_function(2), // 2 parameters
        distrib(distrib),
        params(init)
    {
      nbelem = distrib_size(distrib);
    }
    
    virtual double f (vnl_vector< double > const &x)
    {
      params.k = x[0];
      params.theta = x[1];
      // params.t = x[2];
      // if (alpha <= 0) return FLT_MAX;
      // if (sigma <= 0) return FLT_MAX;
      GammaDistrib model (params);
      double logl = model_log_likelihood(distrib, nbelem, model);
      return -logl;
    }
    
  private:
    const distrib_type& distrib;
    GammaDistrib::Params params;
    double nbelem;
  };


  double estimate_gamma(const std::map<double,double>& distrib,
			GammaDistrib::Params& init)
  {
    gamma_likelihood_function f(distrib, init);
    // vnl_lbfgs minimizer(f);
    // vnl_powell minimizer(&f);
    vnl_amoeba minimizer(f);
    vnl_vector<double> estimate(2);
    estimate[0] = init.k;
    estimate[1] = init.theta;
    // estimate[2] = init.t;
    // Q_ASSERT(minimizer.minimize(estimate));
    minimizer.minimize(estimate);
    init.k = estimate[0];
    init.theta = estimate[1];
    // init.t = estimate[2];
    GammaDistrib m (init);
    double logl = model_log_likelihood(distrib, 
                                       distrib_size(distrib),
                                       m);
    return logl;
  }
                        
#endif

}
