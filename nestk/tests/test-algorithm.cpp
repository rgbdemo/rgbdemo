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

#include <ntk/utils/stl.h>

#include <set>
#include <iostream>
#include <iterator>

struct Predicate
{
  bool operator() (int a, int b) const
  { return 3*a == b || b*3==a; }
};

bool test_keep_maximal()
{
  std::set<int> s;
  s.insert(2);
  s.insert(8);
  s.insert(4);
  s.insert(7);
  s.insert(3);
  s.insert(9);
  s.insert(10);
  s.insert(12);
  
  std::set<int> output;
  Predicate pred;
  ntk::keep_maximal_among_sorted_objects(std::inserter(output, output.begin()), 
                                         s.begin(), s.end(), pred);
  
  if (output.size() != s.size() - 2) return false;
  if (output.find(3) != output.end()) return false;
  if (output.find(4) != output.end()) return false;
  return true;
}

int main()
{
  bool ok = true;
  ok &= test_keep_maximal();
  if (!ok) return 1;
  return 0;
}
