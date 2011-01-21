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

#include "qt_utils.h"

#include <QRegion>
#include <QVector>
#include <QDir>
#include <QStringList>

#include <cstring>

namespace ntk {
  QTextStream qOut (stdout);
  QTextStream qErr (stderr);
  QTextStream qIn (stdin);
}

QTextStream& operator>>(QTextStream& input, bool& b)
{
  int i;
  input >> i;
  b = (i != 0);
  return input;
}

QTextStream& operator<<(QTextStream& output, const bool b)
{
  output << (int) b;
  return output;
}

QTextStream& operator>>(QTextStream& input, unsigned char& c)
{
  input >> (char&) c;
  return input;
}

QTextStream& operator<<(QTextStream& output, const unsigned char c)
{
  output << (char) c;
  return output;
}

namespace ntk
{

  void filesWithExts(QStringList& l, const QDir& dir, 
                     const char* ext1, const char* ext2, 
                     const char* ext3, const char* ext4)
  {
    QStringList exts;
    if (ext1) exts << ext1;
    if (ext2) exts << ext2;
    if (ext3) exts << ext3;
    if (ext4) exts << ext4;

    QDir d = dir;
    d.setFilter(QDir::Files);
    d.setNameFilters(exts);
    l = d.entryList();
  }

} // end of ntk

namespace ntk
{

QRectF fitInRect(const QRectF& rect, const QSizeF& size)
{
  QSizeF s = rect.size();
  s.scale(size, Qt::KeepAspectRatio);
  QRectF r = rect;
  r.setSize(s);
  return r;
}

unsigned qregion_area(const QRegion& r)
{
  unsigned area = 0;
  QVector<QRect> rects = r.rects();
  for (int i = 0; i < rects.size(); ++i)  area += rects[i].width()*rects[i].height();
  return area;
}

void readFullRawData(QLocalSocket& socket, QDataStream& stream, char* data, int len)
{
  int num_bytes = 0;
  while ((stream.status() == QDataStream::Ok) && num_bytes < len)
  {
    socket.waitForReadyRead(100);
    int new_bytes = stream.readRawData(data+num_bytes, len-num_bytes);
    if (new_bytes > 0)
      num_bytes += new_bytes;
  }
}

} // end of ntk
