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

#include "levenberg_marquart_minimizer.h"

#include <ntk/utils/debug.h>

#include <unsupported/Eigen/NonLinearOptimization>

using namespace Eigen;

namespace ntk
{

class EigenFunctor
{
public:
  typedef double Scalar;
  typedef VectorXd InputType;
  typedef VectorXd ValueType;
  typedef Matrix<Scalar,Dynamic,Dynamic> JacobianType;

  enum {
      InputsAtCompileTime = Dynamic,
      ValuesAtCompileTime = Dynamic
  };

public:
  EigenFunctor(ntk::CostFunction& cost_function)
   : m_cost_function(cost_function),
     m_current_input(cost_function.inputDimension()),
     m_current_output(cost_function.outputDimension()),
     m_input_dimension(cost_function.inputDimension()),
     m_output_dimension(cost_function.outputDimension())
  {
  }

  int inputs() const { return m_cost_function.inputDimension(); }
  int values() const { return m_cost_function.outputDimension(); }

  int operator()(const VectorXd &x, VectorXd &fvec) const
  {
    std::copy(x.data(), x.data() + m_input_dimension, m_current_input.begin());
    m_cost_function.evaluate(m_current_input, m_current_output);
    std::copy(m_current_output.begin(), m_current_output.end(), fvec.data());
    return 0;
  }

private:
  mutable CostFunction& m_cost_function;
  mutable std::vector<double> m_current_input;
  mutable std::vector<double> m_current_output;
  int m_input_dimension;
  int m_output_dimension;
};

void LevenbergMarquartMinimizer :: minimize(ntk::CostFunction &f,
                                            std::vector<double> &x)
{
  m_start_error = f.outputNorm(x);

  EigenFunctor eigen_f (f);
  NumericalDiff<EigenFunctor> diff_eigen_f(eigen_f, 1e-7);
  LevenbergMarquardt< NumericalDiff<EigenFunctor> > eigen_minimizer(diff_eigen_f);
  VectorXd eigen_x(f.inputDimension());
  std::copy(x.begin(), x.end(), eigen_x.data());
  int info = eigen_minimizer.minimize(eigen_x);
  m_info = info;
  m_num_function_evaluations = eigen_minimizer.nfev;
  m_num_iterations = eigen_minimizer.iter;
  std::copy(eigen_x.data(), eigen_x.data() + f.inputDimension(), x.begin());

  m_end_error = f.outputNorm(x);
}

void LevenbergMarquartMinimizer::diagnoseOutcome(int debug_level) const
{
  ntk_dbg(debug_level) << cv::format("[LVMQ Status=%i NEval=%i NIter=%i ErrStartEnd=%f/%f]",
                                     m_info, m_num_function_evaluations, m_num_iterations,
                                     m_start_error, m_end_error);
}

} // ntk

