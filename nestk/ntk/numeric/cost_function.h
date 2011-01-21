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

#ifndef NTK_NUMERIC_COST_FUNCTION_H
#define NTK_NUMERIC_COST_FUNCTION_H

#include <ntk/core.h>
#include <vector>

namespace ntk
{

class CostFunction
{
public:
  // Initialize a cost function providing the number of variables
  // of the function to optimize and the number of errors it provides.
  CostFunction(int func_input_dimension, int func_output_dimension)
    : m_input_dim(func_input_dimension),
      m_output_dim(func_output_dimension)
  {}

  virtual void evaluate(const std::vector<double>& input, std::vector<double>& output) const = 0;
  double outputNorm(const std::vector<double>& input) const;

  int inputDimension() const { return m_input_dim; }
  int outputDimension() const { return m_output_dim; }

private:
  int m_input_dim;
  int m_output_dim;
};

class CostFunctionMinimizer
{
public:
  // Minimize the given function taking x as initial value.
  // When returning, x will contain the found minima.
  virtual void minimize(CostFunction& f, std::vector<double>& x) = 0;
  virtual void diagnoseOutcome(int debug_level) const {};
};

}

#endif // NTK_NUMERIC_COST_FUNCTION_H
