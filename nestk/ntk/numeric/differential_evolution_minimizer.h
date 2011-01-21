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

#ifndef NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_MINIMIZER_H
#define NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_MINIMIZER_H

#include <ntk/core.h>
#include <ntk/numeric/cost_function.h>

namespace ntk
{

class DifferentialEvolutionMinimizer : public CostFunctionMinimizer
{
public:
  DifferentialEvolutionMinimizer(int population_size,
                                 int max_generations,
                                 const std::vector<double>& min_values,
                                 const std::vector<double>& max_values)
    : m_population_size(population_size),
      m_max_generations(max_generations),
      m_min_values(min_values),
      m_max_values(max_values)
  {
  }

public:
  virtual void minimize(CostFunction& f, std::vector<double>& x);

private:
  int m_population_size;
  int m_max_generations;
  std::vector<double> m_min_values;
  std::vector<double> m_max_values;
};

} // ntk

#endif // NTK_NUMERIC_DIFFERENTIAL_EVOLUTION_MINIMIZER_H
