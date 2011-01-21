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

#include "differential_evolution_minimizer.h"
#include "differential_evolution_solver.h"

#include <ntk/utils/debug.h>
#include <ntk/numeric/utils.h>

#include <unsupported/Eigen/NonLinearOptimization>

using namespace Eigen;
using namespace ntk;

namespace
{

class CostFunctionSolver : public DifferentialEvolutionSolver
{
public:
  CostFunctionSolver(ntk::CostFunction& cost_function, int population_size)
    : DifferentialEvolutionSolver(cost_function.inputDimension(), population_size),
      m_cost_function(cost_function),
      m_current_input(cost_function.inputDimension()),
      m_current_output(cost_function.outputDimension()),
      m_input_dim(cost_function.inputDimension()),
      m_output_dim(cost_function.outputDimension())
  {}

  virtual double EnergyFunction(double trial[],bool &bAtSolution)
  {
    std::copy(trial, trial + m_input_dim, m_current_input.begin());
    m_cost_function.evaluate(m_current_input, m_current_output);
    double error = 0;
    foreach_idx(i, m_current_output)
    {
      error += ntk::math::sqr(m_current_output[i]);
    }
    return error;
  }

private:
  ntk::CostFunction& m_cost_function;
  std::vector<double> m_current_input;
  std::vector<double> m_current_output;
  int m_input_dim;
  int m_output_dim;
};

}

namespace ntk
{

void DifferentialEvolutionMinimizer :: minimize(ntk::CostFunction &f,
                                                std::vector<double> &x)
{
  ntk_assert(m_min_values.size() == f.inputDimension()
             && m_max_values.size() == f.inputDimension(),
             "min/max values must have input dimension size.");
  CostFunctionSolver solver (f, m_population_size);
  solver.Setup(&m_min_values[0], &m_max_values[0],
               DifferentialEvolutionSolver::stBest1Exp,0.8,0.75);
  solver.Solve(m_max_generations);
  double *solution = solver.Solution();
  std::copy(solution, solution + f.inputDimension(), x.begin());
  ntk_dbg_print(solver.Energy(), 1);
}

} // ntk
