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

#include "screen_grabber.h"

namespace ntk
{

ScreenGrabber :: ScreenGrabber(const std::string& dir_name)
  : m_frame_count(0), m_dir(dir_name.c_str())
{
  m_dir.mkpath(".");
}

void ScreenGrabber :: reset()
{
  m_frame_count = 0;
}

void ScreenGrabber :: saveFrame(const QPixmap& pixmap)
{
  pixmap.save(m_dir.absoluteFilePath(QString("frame%1.png").arg(m_frame_count, 4, 10, QChar('0'))));
  ++m_frame_count;
}

} // ntk
