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
#include <iostream>
#include <map>

typedef std::map<int,int> map_int_int;

using namespace ntk;

bool test1()
{
  std::map<int, int> orig;
  orig[5] = 12;
  orig[8] = 15;
  orig[12] = 8;
  
  std::map<int,int> modified = orig;
  {
    AssociativeContainerTransaction<map_int_int> transaction(modified);
    transaction.changeElement(5) = 15;
    transaction.removeElement(8);
  }
  
  if (modified.size() != 2) return false;
  if (modified[5] != 15) return false;
    
  return true;
}

bool test2()
{
  std::map<int, int> orig;
  orig[5] = 12;
  orig[8] = 15;
  orig[12] = 8;
  foreach_const_it(it, orig, map_int_int) 
    std::cout << "[" << it->first << "] => [" << it->second << "]" << std::endl;
  
  std::map<int,int> modified = orig;
  {
    AssociativeContainerTransaction<map_int_int> transaction(modified);
    transaction.changeElement(5) = 15;
    transaction.changeElement(8) = 12;
    transaction.removeElement(12);
    transaction.abortTransaction();
  }
  foreach_const_it(it, modified, map_int_int) 
    std::cout << "[" << it->first << "] => [" << it->second << "]" << std::endl;
    
  if (modified.size() != 3) return false;
  if (modified[5] != 12) return false;
  if (modified[8] != 15) return false;
  
  return true;
}

int main()
{
  bool ok = true;
  ok &= test1();
  ok &= test2();
  return ok != true;
}
