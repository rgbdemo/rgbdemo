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
#include <ntk/utils/stl.h>
#include <map>
#include <iostream>
#include <string>

using namespace ntk;

int main()
{
  std::map<std::string,std::string> m;
  m["hello"] = "bonjour";
  m["bye"] = "au revoir";
  
  {
    key_iterator< std::map<std::string,std::string>::const_iterator > it (m.begin());
    for (; it != m.end(); ++it)
      std::cout << it->c_str() << std::endl;
  }
  
  {
    value_iterator< std::map<std::string,std::string>::const_iterator > it (m.begin());
    for (; it != m.end(); ++it)
      std::cout << (*it).c_str() << std::endl;
  }
  
  return 0;
}

