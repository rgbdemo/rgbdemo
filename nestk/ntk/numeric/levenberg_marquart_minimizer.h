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

#ifndef NTK_NUMERIC_LEVENBERG_MARQUART_MINIMIZER_H
#define NTK_NUMERIC_LEVENBERG_MARQUART_MINIMIZER_H

#include <ntk/core.h>
#include <ntk/numeric/cost_function.h>

namespace ntk
{

class LevenbergMarquartMinimizer : public CostFunctionMinimizer
{
public:
  virtual void minimize(CostFunction& f, std::vector<double>& x);
  virtual void diagnoseOutcome(int debug_level = 2) const;

private:
  int m_info;
  int m_num_function_evaluations;
  int m_num_iterations;
  double m_start_error;
  double m_end_error;
};

} // ntk

#endif // NTK_NUMERIC_LEVENBERG_MARQUART_MINIMIZER_H
