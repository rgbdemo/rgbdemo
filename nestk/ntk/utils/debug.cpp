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

#include "debug.h"
#include "xml_serializable.h"

#include <iostream>

namespace ntk
{
  int ntk_debug_level = 0;
    
  void assert_failure(const char* where, const char* what, const char* cond)
  {
    std::cerr << "ASSERT failure in " << where << ": " << what
        << " [" << cond << "]" << std::endl;
    abort();
  }
  
  void fatal_error(const char* what, int code)
  {
    std::cerr << "FATAL failure: " << what << std::endl;
    exit(code);
  }

}

const NtkDebug& operator<<(const NtkDebug& d, const std::string& rhs)
{
  d << rhs.c_str();
  return d;
}

const NtkDebug& operator<<(const NtkDebug& d, const ntk::XmlSerializable& rhs)
{ 
  ntk::XMLNode e = ntk::XMLNode::createXMLTopNode("debug");
  rhs.fillXmlElement(e);
  d.stringPtr()->append(e.createXMLString(true));
  return d;
}

