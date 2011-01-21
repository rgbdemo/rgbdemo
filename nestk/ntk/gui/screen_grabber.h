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

#ifndef NTK_GUI_SCREENGRABBER_H
#define NTK_GUI_SCREENGRABBER_H

#include <ntk/core.h>

#include <QPixmap>
#include <QDir>

namespace ntk
{

class ScreenGrabber
{
public:
  ScreenGrabber(const std::string& dir_name);

public:
  void reset();
  void saveFrame(const QPixmap& pixmap);

private:
  int m_frame_count;
  QDir m_dir;
};

} // ntk

#endif // NTK_GUI_SCREENGRABBER_H
