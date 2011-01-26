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

#ifndef   	NTK_UTILS_DEBUG_H_
# define   	NTK_UTILS_DEBUG_H_

# include <ntk/core.h>

# include <QString>
# include <QTextStream>
# include <QDebug>

namespace ntk
{
  extern int ntk_debug_level;
  class XmlSerializable;
}

class NtkDebug
{
  public:
    virtual ~NtkDebug()
    { qDebug() << s; }

  public:
    void print(const ntk::XmlSerializable& rhs) const;   

  public:
    QString* stringPtr() const { return &s; }

  private:
    mutable QString s;
};

#define NTK_DECLARE_DEBUG_OPERATOR(Type) \
inline const NtkDebug& operator<<(const NtkDebug& d, Type rhs) \
{ QTextStream stream(d.stringPtr()); stream << rhs; return d; }

NTK_DECLARE_DEBUG_OPERATOR(bool)
NTK_DECLARE_DEBUG_OPERATOR(short)
NTK_DECLARE_DEBUG_OPERATOR(unsigned short)
NTK_DECLARE_DEBUG_OPERATOR(int)
NTK_DECLARE_DEBUG_OPERATOR(unsigned)
NTK_DECLARE_DEBUG_OPERATOR(long)
NTK_DECLARE_DEBUG_OPERATOR(unsigned long)
NTK_DECLARE_DEBUG_OPERATOR(long long)
NTK_DECLARE_DEBUG_OPERATOR(unsigned long long)
NTK_DECLARE_DEBUG_OPERATOR(double)
NTK_DECLARE_DEBUG_OPERATOR(const QString&)
NTK_DECLARE_DEBUG_OPERATOR(const char*)

const NtkDebug& operator<<(const NtkDebug& d, const std::string& rhs);

const NtkDebug& operator<<(const NtkDebug& d, const ntk::XmlSerializable& rhs);

#ifndef NDEBUG
# define ntk_dbg(level) if (level <= ntk::ntk_debug_level) NtkDebug()
#else
# define ntk_dbg(level) if (0) NtkDebug()
#endif // ndef NDEBUG

# define ntk_log() NtkDebug()
# define ntk_log_error() NtkDebug() << "Error: "

#ifndef NDEBUG
# define ntk_dbg_print(value, level) \
  ntk_dbg(level) << #value": " << value;
#else
# define ntk_dbg_print(value, level)
#endif // ndef NDEBUG

namespace ntk {
  
  void assert_failure(const char* where, const char* what, const char* cond);
  void fatal_error(const char* what, int code=1);

}

#ifdef __GNUC__
# define PRETTY_FUNCTION __PRETTY_FUNCTION__
#else
# define PRETTY_FUNCTION ""
#endif // __GNUC__

#ifndef NDEBUG
# define ntk_assert(cond,what) if (!(cond)) ntk::assert_failure(PRETTY_FUNCTION, what, #cond);
#else
# define ntk_assert(cond,what)
#endif // ndef NDEBUG

#define ntk_ensure(cond,what) if (!(cond)) { ntk::assert_failure(PRETTY_FUNCTION, what, #cond); \
                                             ntk::fatal_error(what); }

#define ntk_dbg_enter_function(level) \
  ntk_dbg(level) << "<Entering> " << PRETTY_FUNCTION \
                 << " (" << this << ")" \
                 << " (" << cv::getThreadNum() << ")";

#define ntk_dbg_leave_function(level) \
  ntk_dbg(level) << "</Leaving> " << PRETTY_FUNCTION \
                 << " (" << this << ")" \
                 << " (" << cv::getThreadNum() << ")";

#endif 	    /* !NTK_UTILS_DEBUG_H_ */
