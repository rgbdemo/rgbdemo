//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef NTK_MESH_UTILS_H
#define NTK_MESH_UTILS_H

# include <ntk/core.h>

# include <vgl/vgl_triangle_scan_iterator.h>

namespace ntk
{

// Apply f to each pixel of a given triangle.
template <typename Func>
void applyToTriangle(Func& f,
                     const int x0, const int y0,
                     const int x1, const int y1,
                     const int x2, const int y2)
{
  vgl_triangle_scan_iterator<int> iterator;
  iterator.a.x = x0; iterator.a.y = y0;
  iterator.b.x = x1; iterator.b.y = y1;
  iterator.c.x = x2; iterator.c.y = y2;
  iterator.reset();
  while (iterator.next())
  {
    int y = iterator.scany();
    for (int x = iterator.startx(); x <= iterator.endx(); ++x)
      f(x,y);
  }
}

// Apply f to each pixel of a given triangle.
template <typename Func>
void applyToTriangleWithBorder(Func& f,
                               const int x0, const int y0,
                               const int x1, const int y1,
                               const int x2, const int y2,
                               int border)
{
  vgl_triangle_scan_iterator<int> iterator;
  iterator.a.x = x0; iterator.a.y = y0;
  iterator.b.x = x1; iterator.b.y = y1;
  iterator.c.x = x2; iterator.c.y = y2;
  iterator.reset();

  if (!iterator.next()) return;

  int y = iterator.scany();
  for (int ystart = y-border; ystart <= y; ++ystart)
    for (int x = iterator.startx()-border; x <= iterator.endx()+border; ++x)
      f(x,ystart);

  int last_y=y;
  int last_startx=iterator.startx();
  int last_endx=iterator.endx();
  while (iterator.next())
  {
    int y = iterator.scany();
    for (int x = iterator.startx()-border; x <= iterator.endx()+border; ++x)
      f(x,y);
    last_y=y;
    last_startx=iterator.startx();
    last_endx=iterator.endx();
  }

  for (int ystart = last_y+1; ystart <= last_y+border; ++ystart)
    for (int x = last_startx-border; x <= last_endx+border; ++x)
      f(x,ystart);
}

}

#endif // NTK_MESH_UTILS_H
