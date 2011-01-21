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

#ifndef TESTS_TEST_COMMON_H
#define TESTS_TEST_COMMON_H

#include <ntk/ntk.h>

#define NTK_TEST_FLOAT_EQ(Value, Ref) test_float_eq(#Value, Value, Ref)

namespace ntk
{

inline bool test_float_eq(const char* value_name, float value, float ref)
{
  ntk_dbg(1) << "[TEST " << value_name << "] " << value << " == " << ref;
  bool ok = (flt_eq(value, ref, 1e-3));
  ntk_ensure(ok, "Test failed");
  return ok;
}

}

#endif // TESTS_TEST_COMMON_H
