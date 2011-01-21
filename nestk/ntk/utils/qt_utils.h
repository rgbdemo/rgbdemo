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

#ifndef   	NTK_QT_UTILS_H_
# define   	NTK_QT_UTILS_H_

# include <ntk/core.h>
# include <ntk/utils/debug.h>

# include <QFile>
# include <QDataStream>
# include <QTextStream>
# include <QHash>
# include <QPoint>
# include <QRectF>
# include <QString>
# include <QPolygonF>
# include <QMatrix>
# include <QLocalSocket>

class QDir;
class QStringList;
class QSizeF; 
class QPixmap; 
class QImage; 
class QRegion;

template <class T> class auto_ptr;

template <class T>
unsigned qHash(const ntk::Ptr<T>& p)
{ return qHash((const T*)p); }

namespace ntk 
{
  template <class Key, class Value>
  const Value& qhash_get(const QHash<Key,Value>& hash, const Key& key)
  {
    typename QHash<Key,Value>::const_iterator it = hash.find(key);
    Q_ASSERT(it != hash.end());
    return it.value();
  }
}

inline unsigned qHash(const QPoint& p)
{ return p.x()+p.y(); }
  
inline bool operator<(const QPoint& lhs, const QPoint& rhs)
{
  if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();
  return lhs.y() < rhs.y();
}


namespace ntk
{

  inline double area(const QRectF& r)
  {
    return r.width()*r.height();
  }

  inline double overlap_ratio(const QRectF& r1, const QRectF& r2)
  {
    QRectF u12 = r1 | r2;
    QRectF i12 = r1 & r2;
    return area(i12)/area(u12);
  }

} // end of ntk

template <class T>
QDataStream& operator >>(QDataStream& is, ntk::Ptr<T>& obj)
{
  T* robj = new T();
  is >> (*robj);
  obj = toPtr(robj);
  return is;
}
  
template <class T>
QDataStream& operator <<(QDataStream& os, const ntk::Ptr<T>& obj)
{
  os << *obj;
  return os;
}

namespace ntk
{

template <class IteratorType, class ValueType>
void toQSet(QSet<ValueType>& output, IteratorType begin, IteratorType end)
{
  while (begin != end)
  {
    output.insert(*begin);
    ++begin;
  }
}

inline
QString toQt(const std::string& s)
{
  return QString(s.c_str());
}

inline
QRectF toQt(const cv::Rect_<float>& rect)
{
  return QRectF(rect.x, rect.y, rect.width, rect.height);
}

inline
QPointF toQt(const cv::Point2f& p)
{
  return QPointF(p.x, p.y);
}

inline
QPoint toQt(const cv::Point2i& p)
{
  return QPoint(p.x, p.y);
}

inline
QRect toQt(const cv::Rect& rect)
{
  return QRect(rect.x, rect.y, rect.width, rect.height);
}

inline cv::Point2i toOpencv(const QPoint& p)
{
  return cv::Point2i(p.x(), p.y());
}

} // end of ntk

QTextStream& operator>>(QTextStream& input, unsigned char& c);
QTextStream& operator<<(QTextStream& input, const unsigned char c);
QTextStream& operator>>(QTextStream& input, bool& b);
QTextStream& operator<<(QTextStream& input, const bool b);

inline
const NtkDebug& operator<<(const NtkDebug& os, const QPointF& p)
{
  os << "[" << p.x() << ", " << p.y() << "]";
  return os;
}

namespace ntk
{

  QRectF fitInRect(const QRectF& rect, const QSizeF& size);
  
  unsigned qregion_area(const QRegion& r);

  extern QTextStream qOut;
  extern QTextStream qErr;
  extern QTextStream qIn;

  template <class T>
  bool qsave_object(const T& obj, const char* filename)
  {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly))
      return false;
    QDataStream out(&file);   // we will serialize the data into the file
    out << obj;
    return out.status() == QDataStream::Ok;
  }

  template <class T>
  bool qload_object(T& obj, const char* filename)
  {
    QFile file(filename);
    if (!file.exists()) return false;
    if (!file.open(QIODevice::ReadOnly))
      return false;
    QDataStream in(&file);   // we will serialize the data into the file
    in >> obj;
    return in.status() == QDataStream::Ok;
  }

  template <class T>
  bool qsave_text(const T& obj, const char* filename)
  {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly))
      return false;
    QTextStream out(&file);   // we will serialize the data into the file
    out << obj;
    return out.status() == QTextStream::Ok;
  }

  template <class T>
  bool qload_text(T& obj, const char* filename)
  {
    QFile file(filename);
    if (!file.exists()) return false;
    if (!file.open(QIODevice::ReadOnly))
      return false;
    QTextStream in(&file);   // we will serialize the data into the file
    in >> obj;
    return in.status() == QTextStream::Ok;
  }
  
  /*!
   * Read len bytes from a datastream.
   * Do not return until all len bytes have been written or stream state
   * is not Ok.
   */
  void readFullRawData(QLocalSocket& socket, QDataStream& stream, char* data, int len);

} // end of ntk

#endif	    /* !NTK_QT_UTILS_H_ */
