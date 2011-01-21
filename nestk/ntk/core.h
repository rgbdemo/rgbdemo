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

#ifndef   	NTK_CORE_H_
# define   	NTK_CORE_H_

# include <limits>

# include <opencv/cxcore.h>
# include <opencv/cv.h>
# include <opencv/highgui.h>

#ifndef FLT_MAX
# define FLT_MAX std::numeric_limits<double>::max()
#endif

#ifndef M_PI
# define M_PI 3.141592653589793238462643
#endif

# include <ntk/utils/common.h>

#endif	    /* !NTK_CORE_H_ */
