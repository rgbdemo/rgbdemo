//
//                Copyright (c) 2000-2003 TargetJr Consortium
//               GE Corporate Research and Development (GE CRD)
//                             1 Research Circle
//                            Niskayuna, NY 12309
//                            All Rights Reserved
//              Reproduction rights limited as described below.
//
//      Permission to use, copy, modify, distribute, and sell this software
//      and its documentation for any purpose is hereby granted without fee,
//      provided that (i) the above copyright notice and this permission
//      notice appear in all copies of the software and related documentation,
//      (ii) the name TargetJr Consortium (represented by GE CRD), may not be
//      used in any advertising or publicity relating to the software without
//      the specific, prior written permission of GE CRD, and (iii) any
//      modifications are clearly marked and summarized in a change history
//      log.
//
//      THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,
//      EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY
//      WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
//      IN NO EVENT SHALL THE TARGETJR CONSORTIUM BE LIABLE FOR ANY SPECIAL,
//      INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY KIND OR ANY
//      DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//      WHETHER OR NOT ADVISED OF THE POSSIBILITY OF SUCH DAMAGES, OR ON
//      ANY THEORY OF LIABILITY ARISING OUT OF OR IN CONNECTION WITH THE
//      USE OR PERFORMANCE OF THIS SOFTWARE.
//

#include "arg.h"

#include <cassert>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cstdlib> // exit()
#include <cmath>   // floor()
#include <vector>
#include <list>

namespace ntk
{
  struct ntk_sprintf : public std::string
  {
    ntk_sprintf(char const *fmt, ...) : std::string("")
    {
      va_list ap;
      va_start(ap, fmt);

      char s[65536];
      vsprintf(s, fmt, ap);
      if (strlen(s) >= sizeof s)
        std::cerr << __FILE__ ": WARNING! Possible memory corruption after call to vsprintf()\n";
      std::string::operator=(s);

      va_end(ap);
    }
    operator char const* () const { return c_str(); }
  };
  std::ostream & operator<<(std::ostream &os,const ntk_sprintf& s)
  {
    return os << (char const*)s;
  }

  std::ostream& ntk_printf(std::ostream& s, char const * fmt, ...)
  {
    char buf[65536];

    va_list ap;
    va_start(ap, fmt);
    vsprintf(buf, fmt, ap);
    va_end(ap);

    return s << buf;
  }

}

namespace ntk
{

//------------------------------------------------------------------------------

char const* arg_base::option()
{ return option_.c_str(); }

char const* arg_base::help()
{ return help_.c_str(); }

//: Parse the list of arguments....
void arg_parse(int& argc, char **& argv,
                   bool warn_about_unrecognized_arguments)
{
  arg_base::parse_deprecated(argc, argv,
                                 warn_about_unrecognized_arguments);
}

//: Add an externally supplied list of args to the global list.
void arg_include(arg_info_list& l)
{
  arg_base::include_deprecated(l);
}

//: Print all args, and usage messages.
void arg_display_usage_and_exit(char const* msg)
{
  arg_base::display_usage_and_exit(msg);
}


//: Returns true if arg was set on the command line.
bool arg_base::set() const
{ return set_; }

static arg_info_list& current_list() // instance "method"
{
  static arg_info_list list;
  return list;
}

//: Add another arg_info_list to the current one.
// This allows for the inclusion of different sets of arguments into the
// main program, from different libraries.
void arg_base::include_deprecated(arg_info_list& l)
{
  current_list().include(l);
}

//
void arg_base::add_to_current(arg_base* a)
{
  current_list().add(a);
}

//: The main static method.
void arg_base::parse_deprecated(int& argc, char **& argv, bool warn_about_unrecognized_arguments)
{
  current_list().parse(argc, argv, warn_about_unrecognized_arguments);
}

void arg_base::set_help_option(char const* str)
{
  current_list().set_help_option( str);
}

void arg_base::set_help_precis(char const* str)
{
  current_list().set_help_precis( str);
}

void arg_base::set_help_description(char const* str)
{
  current_list().set_help_description( str);
}

void arg_base::display_usage(char const* msg)
{
  if (msg) std::cerr << "** WARNING ** " << msg << std::endl;
  current_list().display_help("");
}

void arg_base::display_usage_and_exit(char const* msg)
{
  if (msg) std::cerr << "** ERROR ** " << msg << std::endl;
  current_list().display_help("");
  std::exit(-1);
}

// arg_base constructors

arg_base::arg_base(arg_info_list& l, char const* option_string, char const* helpstring)
: option_(option_string?option_string:"\0"),
  help_(helpstring?helpstring:"\0")
{
  l.add(this);
}

arg_base::arg_base(char const* option_string, char const* helpstring)
: option_(option_string?option_string:"\0"),
  help_(helpstring?helpstring:"\0")
{
  current_list().add(this);
}

//------------------------------------------------------------------------------

//: Change the help operator (defaults to -?)
void arg_info_list::set_help_option(char const* str)
{
  // check that the operator isn't already being used
  for (unsigned int i=0; i<args_.size(); i++) {
    if (std::strcmp(args_[i]->option(),str) == 0) {
      std::cerr << "arg_info_list: WARNING: requested help operator already assigned\n";
      return;
    }
  }

  help_ = str;
}


//: Add an argument to the list.
void arg_info_list::add(arg_base* argmt)
{
  if ( argmt->option() && help_ == argmt->option() )
    std::cerr << "arg_info_list: WARNING: '-" << help_
             << "' option reserved and will be ignored\n";
  else
    args_.push_back(argmt);
}

//: Append another list.  The other list is not copied, just pointed to.
void arg_info_list::include(arg_info_list& l)
{
  assert(&l != this);

  for (unsigned int i = 0; i < l.args_.size(); ++i)
    add(l.args_[i]);
}

//: Display help about each option in the arg list.
// Note that this function does not exit at the end.
void arg_info_list::display_help( char const*progname)
{
  if (progname)
    std::cerr << "Usage: " << progname << ' ';
  else
    std::cerr << "Usage: aprog ";

  // Print "prog [-a int] string string"
  for (unsigned int i=0; i< args_.size(); i++) {
    if (args_[i]->option()) {
      std::cerr << '[' << args_[i]->option();
      if (std::strlen(args_[i]->type_)> 0)
        std::cerr << ' ' << args_[i]->type_;
      std::cerr << "] ";
    } else {
      // options without switches are required.
      std::cerr << args_[i]->type_ << ' ';
    }
  }

  std::cerr << std::endl << command_precis_ << std::endl;

  // Find longest option, type name, or default
  int maxl_option  = std::max(std::size_t(8), help_.size()); // Length of "REQUIRED" or help option
  int maxl_type    = 4; // Length of "Type", minimum "bool"
  //  int maxl_default = 0;
  for (unsigned int i=0; i< args_.size(); i++)
    if (!args_[i]->help_.empty()) {
      if (!args_[i]->option_.empty()) {
        int l = std::strlen(args_[i]->option());
        if (l > maxl_option) maxl_option = l;
      }
      int l = std::strlen(args_[i]->type_);
      if (l > maxl_type) maxl_type = l;
    }

  // Print long form of args
  std::string fmtbuf = ntk_sprintf("%%%ds %%-%ds %%s ", maxl_option, maxl_type);

  // Do required args first
  std::cerr << "REQUIRED:\n";
  for (unsigned int i=0; i< args_.size(); i++)
    if (!args_[i]->help_.empty())
      if (args_[i]->option_.empty()) {
        ntk_printf(std::cerr, fmtbuf.c_str(), "", args_[i]->type_, args_[i]->help_.c_str());
        std::cerr << " ["; args_[i]->print_value(std::cerr); std::cerr << "]\n"; // default
      }
  std::cerr << std::endl;

  // Then others
  ntk_printf(std::cerr, "Optional:\n");
  ntk_printf(std::cerr, fmtbuf.c_str(), "Switch", "Type", "Help [default value]") << std::endl << std::endl;
  for (unsigned int i=0; i< args_.size(); i++)
    if (!args_[i]->help_.empty())
      if (!args_[i]->option_.empty()) {
        ntk_printf(std::cerr, fmtbuf.c_str(), args_[i]->option(), args_[i]->type_, args_[i]->help_.c_str());
        std::cerr << " ["; args_[i]->print_value(std::cerr); std::cerr << "]\n"; // default
      }
  ntk_printf(std::cerr, fmtbuf.c_str(), help_.c_str(), "bool", "Print this message\n");

  if (!description_.empty()) std::cerr << '\n' << description_;
}

//: Parse the command line, using the current list of args.
//  Remove all recognised arguments from the command line by modifying argc and argv.
void arg_info_list::parse(int& argc, char **& argv, bool warn_about_unrecognized_arguments)
{
  std::vector<bool> done_once(args_.size(), false);

  // 0. Check that there are no duplicate switches, O(n^2) as n is tiny.
  for (unsigned int i = 0; i < args_.size(); ++i)
    if (!args_[i]->option_.empty())
      for (unsigned int j = i+1; j < args_.size(); ++j)
        if (args_[i]->option_ == args_[j]->option_)
          std::cerr << "arg_info_list: WARNING: repeated switch ["
                   << args_[j]->option_ << "]\n";

  // 0a. Clear "set" flags on args
  for (unsigned int i = 0; i < args_.size(); ++i)
    args_[i]->set_ = false;

  // Generate shorter command name
  char * cmdname = argv[0]+std::strlen(argv[0]);
  while (cmdname > argv[0] && *cmdname != '/' && *cmdname != '\\') --cmdname ;
  if (*cmdname == '\\' || *cmdname == '/') cmdname++;


  // 1. Collect option arguments (i.e. ones with "-"),
  // and squeeze them out of argv.
  // Make sure to do things sequentially

  char ** my_argv = argv + 1; // Skip program name
  while (*my_argv) {
    char* argmt = *my_argv;
    bool eaten = false;
    for (unsigned int i = 0; i < args_.size(); ++i) {
      if (!args_[i]->option_.empty()) {
        if ( help_ == argmt ) { // look for the '-?' operator (i.e. HELP)
          display_help(cmdname);
          std::exit(1);
        }

        if (args_[i]->option_==argmt) {
          done_once[i] = true;
          int advance = args_[i]->parse(my_argv + 1);
          args_[i]->set_ = true;
          if (advance >= 0) {
            // Pull down remaining args
            for (char ** av = my_argv; *(av + advance); ++av)
              *av = *(av + advance + 1);

            eaten = true;
            break;
          }
        }
      }
    }
    if (!eaten)
      ++my_argv;
  }

  if (verbose_) {
    std::cerr << "args remaining:";
    for (char ** av = argv; *av; ++av)
      std::cerr << " [" << *av << ']';
    std::cerr << std::endl;
  }


  // 2. Just take from the list to fill the non-option arguments
  my_argv = argv + 1;
  int num_satisfied = 0;
  for (unsigned int i = 0; i < args_.size(); ++i)
    if (args_[i]->option_.empty())
    {
      if (*my_argv) {
        done_once[i] = true;
        int advance = args_[i]->parse(my_argv);
        args_[i]->set_ = true;
        my_argv += advance;
        ++num_satisfied;
      } else {
        display_help(cmdname);

        std::cerr << "\nargParse::ERROR: Required arg " << (num_satisfied+1)
                 << " not supplied\n\n";
        std::exit(1);
      }
    }


  // 3. Move my_argv down to first unused arg, and reset argc
  argc = 1;
  for (char ** av = my_argv; *av; ++av)
    ++argc;
  for (int i = 1; i < argc; ++i)
    argv[i] = my_argv[i-1];
  argv[argc] = 0;

  // 4. Error checking.
  //
  // 4.2 Sometimes it's bad if all args weren't used (i.e. trailing args)
  if (autonomy_ == all) {
    std::cerr << "arg_info_list: Some arguments were unused: ";
    for (char ** av = argv; *av; ++av)
      std::cerr << ' ' << *av;
    std::cerr << std::endl;
    display_help(cmdname);
  }

  // 4.3 It's often bad if a switch was not recognized.
  if (warn_about_unrecognized_arguments)
    for (char ** av = argv; *av; ++av)
      if (**av == '-') {
        display_help(cmdname);
        std::cerr << "arg_info_list: WARNING: Unparsed switch [" << *av << "]\n";
      }

  // 5. Some people like a chatty program.
#ifdef DEBUG //fsm: do not print outcome - it looks like an error message.
  if (verbose_) {
    // Print outcome
    for (unsigned int i = 0; i < args_.size(); ++i)
      if (args[i]->option_) {
        std::cerr << "Switch " << args_[i]->option_ << ": "
                 << (!done_once[i] ? "not " : "") << "done, value [";
        args[i]->print_value(std::cerr);
        std::cerr << "]\n";
      }

    for (unsigned int i = 0; i < args.size(); ++i)
      if (!args[i]->option_) {
        std::cerr << "Trailer: ";
        args_[i]->print_value(std::cerr);
        std::cerr << std::endl;
      }

    std::cerr << "args remaining [argc = " << argc << "]:";
    for (char ** av = argv; *av; ++av)
      std::cerr << ' ' << *av;
    std::cerr << "\n--------------\n";
  }
#endif
}


//------------------------------------------------------------------------------

//: function to parse matlab or UNIX style integer ranges.
// eg. 1:3 is matlab for 1,2,3 and 1-3 is UNIX for 1,2,3
//
// parsed as follows:
//   any character other than '-' and ':' is considered a list separator
//   simple ranges can be written as 1:3 or 1-3 (=1,2,3) or 3:1 (=3,2,1)
//   complete ranges can be written as 1:2:5 or 1-2-5 (=1,3,5)
//   negative numbers are handled 'transparently'
//   (e.g. -1:-3 or -1--3 or even -1--1--3 ...:).
//
// Returns 1 on success and 0 on failure.
//
static int list_parse(std::list<int> &out, char ** argv)
{
  out.clear();

  // Empty list specified as the last argument.
  if ( !argv[0] ) return 0; // failure

  std::string str(argv[0]);

#define REGEXP_INTEGER "\\-?[0123456789]+"

  reg_exp range_regexp("(" REGEXP_INTEGER ")"      // int
                           "([:-]" REGEXP_INTEGER ")?" // :int [optional]
                           "([:-]" REGEXP_INTEGER ")?" // :int [optional]
                          );

  while (str.length() > 0 && range_regexp.find(str)) {
    // the start/end positions (ref from 0) of the
    //    current ',' separated token.
    std::ptrdiff_t start= range_regexp.start(0);
    std::ptrdiff_t endp = range_regexp.end(0);
    if (start != 0) {
      std::cerr << "arg<std::list<int> >: Bad argument [" << argv[0] << "]\n";
      return 0; // failure
    }

#ifdef DEBUG
    // this is the current token.
    std::string token = str.substr(start, endp);
    std::cerr << "token = " << token << '\n';
#endif
    std::string match1 = range_regexp.match(1);
#ifdef DEBUG
    std::cerr << "match1 = " << match1 << '\n';
#endif
    std::string match2 = range_regexp.match(2);
#ifdef DEBUG
    std::cerr << "match2 = " << match2 << '\n';
#endif
    std::string match3 = range_regexp.match(3);
#ifdef DEBUG
    std::cerr << "match3 = " << match3 << '\n';
#endif

    // Remove this match from the front of string.
    str.erase(0, endp + 1);

#if 0
    std::cerr << "Range regexp matched [" << token <<  "]: parts ["
             << match1<<"] ["<<match2<<"] ["<<match3<<"]\n"
             << "  str->[" << str << "]\n";
#endif

    bool matched2 = range_regexp.match(2).size() > 0;
    bool matched3 = range_regexp.match(3).size() > 0;

    int s = atoi(match1.c_str());
    int d = 1;
    int e = s;
    if (matched3) {
      // "1:2:10"
      d = atoi(match2.substr(1).c_str());
      e = atoi(match3.substr(1).c_str());
    }
    else if (matched2)
      e = atoi(match2.substr(1).c_str());

#ifdef DEBUG
    std::cerr << "  " << s << ':' << d << ':' << e << '\n';
#endif
    if (e >= s) {
      if (d < 0) {
        std::cerr << "WARNING: d < 0\n";
        d = -d;
      }
      for (int i = s; i <= e; i += d)
        out.push_back(i);
    } else {
      if (d > 0) {
        std::cerr << "WARNING: d > 0\n";
        d = -d;
      }
      for (int i = s; i >= e; i += d)
        out.push_back(i);
    }
  }

  return 1; // success
}

//------------------------------------------------------------------------------

// specializations for specific types.
// In emacs, C-s for "//: unsigned" to find the implementation for arg<unsigned>
// In vi: "/^\/\/: unsigned"

#if 1
# define VDS template <>
#else
# define VDS /* template <> */
#endif

//: bool
VDS void settype(arg<bool> &argmt) { argmt.type_ = "bool"; }

VDS void print_value(std::ostream &s, arg<bool> const &argmt)
{ s << (argmt() ? "set" : "not set"); }

VDS int parse(arg<bool>* argmt, char ** /*argv*/)
{
  argmt->value_ = true;
  return 0; // bool sucks zero args, most others take one.
}

template class arg<bool>;

//: int
VDS void settype(arg<int> &argmt) { argmt.type_ = "integer"; }

VDS void print_value(std::ostream  &s, arg<int> const &argmt)
{ s << argmt(); }

VDS int parse(arg<int>* argmt, char ** argv)
{
  if ( !argv ||  !argv[0] ) {
    // no input
    std::cerr << "arg_parse: Expected integer, none is provided.\n";
    return -1;
  }

  char* endptr = 0;
  double v = std::strtod(argv[0], &endptr);
  if (*endptr != '\0') {
    // There is junk after the number, or no number was found
    std::cerr << "arg_parse: WARNING: Attempt to parse \"" << *argv << "\" as int\n";
    return -1;
  }
  if (v != std::floor(v)) {
    std::cerr << "arg_parse: Expected integer: saw " << argv[0] << std::endl;
    return -1;
  }
  argmt->value_ = int(v);
  return 1;
}

template class arg<int>;

//: unsigned
VDS void settype(arg<unsigned> &argmt) { argmt.type_ = "integer"; }

VDS void print_value(std::ostream &s, arg<unsigned> const &argmt)
{ s << argmt(); }

VDS int parse(arg<unsigned>* argmt, char ** argv)
{
  if ( !argv ||  !argv[0] ) {
    // no input
    std::cerr << "arg_parse: Expected integer, none is provided.\n";
    return -1;
  }

  char* endptr = 0;
  double v = std::strtod(argv[0], &endptr);
  if (*endptr != '\0') {
    // There is junk after the number, or no number was found
    std::cerr << "arg_parse: WARNING: Attempt to parse " << *argv << " as int\n";
    return -1;
  }
  if (v != std::floor(v)) {
    std::cerr << "arg_parse: Expected integer: saw " << argv[0] << std::endl;
    return -1;
  }
  argmt->value_ = unsigned(v);
  return 1;
}

template class arg<unsigned>;

//: float
VDS void settype(arg<float> &argmt) { argmt.type_ = "float"; }

VDS void print_value(std::ostream &s, arg<float> const &argmt)
{ s << argmt(); }

VDS int parse(arg<float>* argmt, char ** argv)
{
  if ( !argv ||  !argv[0] ) {
    // no input
    std::cerr << "arg_parse: Expected floating number, none is provided.\n";
    return -1;
  }

  char* endptr = 0;
  argmt->value_ = (float)std::strtod(argv[0], &endptr);
  if (*endptr == '\0')
    return 1;
  // There is junk after the number, or no number was found
  std::cerr << "arg_parse: WARNING: Attempt to parse " << *argv << " as float\n";
  return -1;
}

template class arg<float>;

//: double
VDS void settype(arg<double> &argmt) { argmt.type_ = "float"; }

VDS void print_value(std::ostream &s, arg<double> const &argmt)
{ s << argmt(); }

VDS int parse(arg<double>* argmt, char ** argv)
{
  if ( !argv ||  !argv[0] ) {
    // no input
    std::cerr << "arg_parse: Expected floating number, none is provided.\n";
    return -1;
  }

  char* endptr = 0;
  argmt->value_ = std::strtod(argv[0], &endptr);
  if (*endptr == '\0')
    return 1;
  // There is junk after the number, or no number was found
  std::cerr << "arg_parse: WARNING: Attempt to parse " << *argv << " as double\n";
  return -1;
}

template class arg<double>;

//: char *
VDS void settype(arg<char *> &argmt) { argmt.type_ = "string"; }

VDS void print_value(std::ostream &s, arg<char *> const &argmt)
{ s << '\'' << (argmt()?argmt():"(null)") << '\''; }

VDS int parse(arg<char*>* argmt, char ** argv)
{
  argmt->value_ = argv[0]; // argv is valid till the end of the program so
  return 1;                // it's ok to just grab the pointer.
}

template class arg<char*>;

//: char const *
VDS void settype(arg<char const *> &argmt) { argmt.type_ = "string"; }

VDS void print_value(std::ostream &s, arg<char const *> const &argmt)
{ s << '\'' << (argmt()?argmt():"(null)") << '\''; }

VDS int parse(arg<char const *>* argmt, char ** argv)
{
  if ( !argv ||  !argv[0] ) {
    // no input
    std::cerr << "arg_parse: Expected string, none is provided.\n";
    return -1;
  }

  argmt->value_ = argv[0]; // argv is valid till the end of the program so
  return 1;                // it's ok to just grab the pointer.
}

template class arg<char const*>;

//: std::string
VDS void settype(arg<std::string> &argmt) { argmt.type_ = "string"; }

VDS void print_value(std::ostream &s, arg<std::string> const &argmt)
{ s << '\'' << argmt() << '\''; }

VDS int parse(arg<std::string>* argmt, char ** argv)
{
  if ( !argv ||  !argv[0] ) {
    // no input
    std::cerr << "arg_parse: Expected string, none is provided.\n";
    return -1;
  }

  if (argv[0]) {
    argmt->value_ = argv[0];
    return 1;
  }
  else {
    std::cerr << __FILE__ ": no argument to string option\n";
    return 0;
  }
}

template class arg<std::string>;

//: std::vector<std::string>
VDS void settype(arg<std::vector<std::string> > &argmt) { argmt.type_ = "string list"; }

VDS void print_value(std::ostream &s, arg<std::vector<std::string> > const &argmt)
{
  for (std::vector<std::string>::const_iterator i=argmt().begin(); i!=argmt().end(); ++i)
    s << ' ' << *i;
}

VDS int parse(arg<std::vector<std::string> >* argmt, char ** argv)
{
  if (!argv)
  {
    std::cerr << "arg_parse: Expected string, none is provided.\n";
    return -1;
  }

  int i = 0;
  while (argv[i] != 0)
  {
    argmt->value_.push_back(argv[i]);
    ++i;
  }
  return 1;
}

template class arg< std::vector<std::string> >;

//: std::list<int>
VDS void settype(arg<std::list<int> > &argmt) { argmt.type_ = "integer list"; }

VDS void print_value(std::ostream &s, arg<std::list<int> > const &argmt)
{
  for (std::list<int>::const_iterator i=argmt().begin(); i!=argmt().end(); ++i)
    s << ' ' << *i;
}

VDS int parse(arg<std::list<int> >* argmt, char ** argv)
{
  return list_parse(argmt->value_,argv);
}

template class arg<std::list<int> >;

//: std::vector<int>
VDS void settype(arg<std::vector<int> > &argmt) { argmt.type_ = "integer list"; }

VDS void print_value(std::ostream &s, arg<std::vector<int> > const &argmt)
{
  for (unsigned int i=0; i<argmt().size(); ++i)
    s << ' ' << argmt()[i];
}

VDS int parse(arg<std::vector<int> >* argmt, char ** argv)
{
  std::list<int> tmp;
  int retval = list_parse(tmp,argv);
  // Defaults should be cleared when the user supplies a value
  argmt->value_.clear();
  for (std::list<int>::iterator i=tmp.begin(); i!=tmp.end(); ++i)
    argmt->value_.push_back( *i );
  return retval;
}

template class arg<std::vector<int> >;

//: std::vector<unsigned>
VDS void settype(arg<std::vector<unsigned> > &argmt){argmt.type_="integer list";}

VDS void print_value(std::ostream &s, arg<std::vector<unsigned> > const &argmt)
{
  for (unsigned int i=0; i<argmt().size(); ++i)
    s << ' ' << argmt()[i];
}

VDS int parse(arg<std::vector<unsigned> >* argmt, char ** argv)
{
  std::list<int> tmp;
  int retval = list_parse(tmp,argv);
  // Defaults should be cleared when the user supplies a value
  argmt->value_.clear();
  for (std::list<int>::iterator i=tmp.begin(); i!=tmp.end(); ++i)
    argmt->value_.push_back( unsigned(*i) );
  return retval;
}

template class arg<std::vector<unsigned> >;

//: std::vector<double>
VDS void settype(arg<std::vector<double> > &argmt) {argmt.type_ = "double list";}

VDS void print_value(std::ostream &s, arg<std::vector<double> > const &argmt)
{
  for (unsigned int i=0; i<argmt().size(); ++i)
    s << ' ' << argmt()[i];
}

VDS int parse(arg<std::vector<double> >* argmt, char ** argv)
{
  if ( !argv ||  !argv[0] ) {
    // no input
    std::cerr << "arg_parse: Expected a vector of floating number, none is provided.\n";
    return -1;
  }

  int sucked = 0;
  // Defaults should be cleared when the user supplies a value
  argmt->value_.clear();
  char *current = argv[0];
  while (current) {
    char* endptr = 0;
    double tmp = std::strtod(current, &endptr);
    //argmt->value_
    if (*endptr == '\0') {
      argmt->value_.push_back(tmp);
      ++ sucked;
      ++ argv;
      current = argv[0];
    }
    else if (*endptr == ',')
    {
      argmt->value_.push_back(tmp);
      current = endptr+1;
    }
    else if (endptr == current)
      break; // OK. end of list of doubles.
    else {
      // There is junk after the number, or no number was found
      std::cerr << "arg_parse: WARNING: Attempt to parse " << current << " as double\n";
      return -1;
    }
  }
  return sucked;
}

template class arg<std::vector<double> >;

} // end of ntk

// Regexp part.
namespace ntk
{

//: Copies the given regular expression.

reg_exp::reg_exp (reg_exp const& rxp)
{
  int ind;
  this->progsize = rxp.progsize;      // Copy regular expression size
  this->program = new char[this->progsize]; // Allocate storage
  for (ind=this->progsize; ind-- != 0;)   // Copy regular expresion
  this->program[ind] = rxp.program[ind];
  this->startp[0] = rxp.startp[0];      // Copy pointers into last
  this->endp[0] = rxp.endp[0];        // Successful "find" operation
  this->regmust = rxp.regmust;        // Copy field
  if (rxp.regmust != NULL) {
  char* dum = rxp.program;
  ind = 0;
  while (dum != rxp.regmust) {
    ++dum;
    ++ind;
  }
  this->regmust = this->program + ind;
  }
  this->regstart = rxp.regstart;      // Copy starting index
  this->reganch = rxp.reganch;        // Copy remaining private data
  this->regmlen = rxp.regmlen;        // Copy remaining private data
}


//: Returns true if two regular expressions have the same compiled program for pattern matching.

bool reg_exp::operator== (reg_exp const& rxp) const
{
  if (this != &rxp) {           // Same address?
  int ind = this->progsize;     // Get regular expression size
  if (ind != rxp.progsize)      // If different size regexp
    return false;               // Return failure
  while (ind-- != 0)            // Else while still characters
    if (this->program[ind] != rxp.program[ind])// If regexp are different
      return false;             // Return failure
  }
  return true;                  // Else same, return success
}


//: Returns true if have the same compiled regular expressions and the same start and end pointers.

bool reg_exp::deep_equal (reg_exp const& rxp) const
{
  int ind = this->progsize;     // Get regular expression size
  if (ind != rxp.progsize)      // If different size regexp
  return false;                 // Return failure
  while (ind-- != 0)            // Else while still characters
  if (this->program[ind] != rxp.program[ind]) // If regexp are different
    return false;               // Return failure
  return this->startp[0] == rxp.startp[0] &&  // Else if same start/end ptrs,
       this->endp[0] == rxp.endp[0];    // Return true
}


// The remaining code in this file is derived from the  regular expression code
// whose  copyright statement appears  below.  It has been  changed to work
// with the class concepts of C++ and COOL.

//
// compile and find
//
// Copyright (c) 1986 by University of Toronto.
// Written by Henry Spencer.  Not derived from licensed software.
//
// Permission is granted to anyone to use this software for any
// purpose on any computer system, and to redistribute it freely,
// subject to the following restrictions:
//
// 1. The author is not responsible for the consequences of use of
//  this software, no matter how awful, even if they arise
//  from defects in it.
//
// 2. The origin of this software must not be misrepresented, either
//  by explicit claim or by omission.
//
// 3. Altered versions must be plainly marked as such, and must not
//  be misrepresented as being the original software.
//
// Beware that some of this code is subtly aware of the way operator
// precedence is structured in regular expressions.  Serious changes in
// regular-expression syntax might require a total rethink.
//

//
// The "internal use only" fields in regexp.h are present to pass info from
// compile to execute that permits the execute phase to run lots faster on
// simple cases.  They are:
//
// regstart  char that must begin a match; '\0' if none obvious
// reganch   is the match anchored (at beginning-of-line only)?
// regmust   string (pointer into program) that match must include, or NULL
// regmlen   length of regmust string
//
// Regstart and reganch permit very fast decisions on suitable starting points
// for a match, cutting down the work a lot.  Regmust permits fast rejection
// of lines that cannot possibly match.  The regmust tests are costly enough
// that compile() supplies a regmust only if the r.e. contains something
// potentially expensive (at present, the only such thing detected is * or +
// at the start of the r.e., which can involve a lot of backup).  Regmlen is
// supplied because the test in find() needs it and compile() is computing
// it anyway.
//

//
// Structure for regexp "program".  This is essentially a linear encoding
// of a nondeterministic finite-state machine (aka syntax charts or
// "railroad normal form" in parsing technology).  Each node is an opcode
// plus a "next" pointer, possibly plus an operand.  "Next" pointers of
// all nodes except BRANCH implement concatenation; a "next" pointer with
// a BRANCH on both ends of it is connecting two alternatives.  (Here we
// have one of the subtle syntax dependencies:  an individual BRANCH (as
// opposed to a collection of them) is never concatenated with anything
// because of operator precedence.)  The operand of some types of node is
// a literal string; for others, it is a node leading into a sub-FSM.  In
// particular, the operand of a BRANCH node is the first node of the branch.
// (NB this is *not* a tree structure:  the tail of the branch connects
// to the thing following the set of BRANCHes.)  The opcodes are:
//

// definition   number  opnd?   meaning
#define END   0  // no   End of program.
#define BOL   1  // no   Match "" at beginning of line.
#define EOL   2  // no   Match "" at end of line.
#define ANY   3  // no   Match any one character.
#define ANYOF   4  // str  Match any character in this string.
#define ANYBUT  5  // str  Match any character not in this string.
#define BRANCH  6  // node Match this alternative, or the next...
#define BACK  7  // no   Match "", "next" ptr points backward.
#define EXACTLY 8  // str  Match this string.
#define NOTHING 9  // no   Match empty string.
#define STAR  10   // node Match this (simple) thing 0 or more times.
#define PLUS  11   // node Match this (simple) thing 1 or more times.
#define OPEN  20   // no   Mark this point in input as start of #n.
// OPEN+1 is number 1, etc.
#define CLOSE   30   // no   Analogous to OPEN.

//
// Opcode notes:
//
// BRANCH     The set of branches constituting a single choice are hooked
//        together with their "next" pointers, since precedence prevents
//        anything being concatenated to any individual branch.  The
//        "next" pointer of the last BRANCH in a choice points to the
//        thing following the whole choice.  This is also where the
//        final "next" pointer of each individual branch points; each
//        branch starts with the operand node of a BRANCH node.
//
// BACK     Normal "next" pointers all implicitly point forward; BACK
//        exists to make loop structures possible.
//
// STAR,PLUS  '?', and complex '*' and '+', are implemented as circular
//        BRANCH structures using BACK.  Simple cases (one character
//        per match) are implemented with STAR and PLUS for speed
//        and to minimize recursive plunges.
//
// OPEN,CLOSE   ...are numbered at compile time.
//

//
// A node is one char of opcode followed by two chars of "next" pointer.
// "Next" pointers are stored as two 8-bit pieces, high order first.  The
// value is a positive offset from the opcode of the node containing it.
// An operand, if any, simply follows the node.  (Note that much of the
// code generation knows about this implicit relationship.)
//
// Using two bytes for the "next" pointer is vast overkill for most things,
// but allows patterns to get big without disasters.
//

#define OP(p)       (*(p))
#define NEXT(p)     (((*((p)+1)&0377)<<8) + (*((p)+2)&0377))
#define OPERAND(p)    ((p) + 3)

const unsigned char MAGIC = 0234;

//
// Utility definitions.
//
#define UCHARAT(p)    ((const unsigned char*)(p))[0]

#define FAIL(m) { regerror(m); return NULL; }
#define ISMULT(c)     ((c) == '*' || (c) == '+' || (c) == '?')
#define META  "^$.[()|?+*\\"

//
// Flags to be passed up and down.
//
#define HASWIDTH    01    // Known never to match null string.
#define SIMPLE      02    // Simple enough to be STAR/PLUS operand.
#define SPSTART     04    // Starts with * or +.
#define WORST       0     // Worst case.


//: Return an expression that will match precisely c
// The returned string is owned by the function, and
// will be overwritten in subsequent calls.
const char * reg_exp::protect(char c)
{
  //: This should be in thread local storage.
  static char pattern[3];

  if (std::strchr(META, c) != 0)
  {
    pattern[0] = '\\';
    pattern[1] = c;
    pattern[2] = 0;
  }
  else
  {
    pattern[0] = c;
    pattern[1] = 0;
  }
  return pattern;
}


/////////////////////////////////////////////////////////////////////////
//
//  COMPILE AND ASSOCIATED FUNCTIONS
//
/////////////////////////////////////////////////////////////////////////


//
// Global work variables for compile().
//
static const char* regparse; // Input-scan pointer.
static       int   regnpar; // () count.
static       char  regdummy;
static       char* regcode; // Code-emit pointer; &regdummy = don't.
static       long  regsize; // Code size.

//
// Forward declarations for compile()'s friends.
//
static       char* reg (int, int*);
static       char* regbranch (int*);
static       char* regpiece (int*);
static       char* regatom (int*);
static       char* regnode (char);
static const char* regnext (register const char*);
static       char* regnext (register char*);
static void        regc (unsigned char);
static void        reginsert (char, char*);
static void        regtail (char*, const char*);
static void        regoptail (char*, const char*);


//
// We can't allocate space until we know how big the compiled form will be,
// but we can't compile it (and thus know how big it is) until we've got a
// place to put the code.  So we cheat:  we compile it twice, once with code
// generation turned off and size counting turned on, and once "for real".
// This also means that we don't allocate space until we are sure that the
// thing really will compile successfully, and we never have to move the
// code and thus invalidate pointers into it.  (Note that it has to be in
// one piece because free() must be able to free it all.)
//
// Beware that the optimization-preparation code in here knows about some
// of the structure of the compiled regexp.
//


//: Compile a regular expression into internal code for later pattern matching.

void reg_exp::compile (char const* exp)
{
  register const char*   scan;
  register const char*   longest;
  register unsigned long len;
           int           flags;

  if (exp == NULL) {
    //RAISE Error, SYM(reg_exp), SYM(No_Expr),
    std::cout << "reg_exp::compile(): No expression supplied.\n";
    return;
  }

  // First pass: determine size, legality.
  regparse = exp;
  regnpar = 1;
  regsize = 0L;
  regcode = &regdummy;
  regc(MAGIC);
  if (!reg(0, &flags))
  {
    std::cout << "reg_exp::compile(): Error in compile.\n";
    return;
  }
  this->startp[0] = this->endp[0] = this->searchstring = NULL;

  // Small enough for pointer-storage convention?
  if (regsize >= 32767L) // Probably could be 65535L.
  {
    //RAISE Error, SYM(reg_exp), SYM(Expr_Too_Big),
    std::cout << "reg_exp::compile(): Expression too big.\n";
    return;
  }

  // Allocate space.
//#ifndef std::WIN32
  if (this->program != NULL) delete [] this->program;
//#endif
  this->program = new char[regsize];
  this->progsize = (int) regsize;

  if (this->program == NULL) {
    //RAISE Error, SYM(reg_exp), SYM(Out_Of_Memory),
    std::cout << "reg_exp::compile(): Out of memory.\n";
    return;
  }

  // Second pass: emit code.
  regparse = exp;
  regnpar = 1;
  regcode = this->program;
  regc(MAGIC);
  reg(0, &flags);

  // Dig out information for optimizations.
  this->regstart = '\0'; // Worst-case defaults.
  this->reganch = 0;
  this->regmust = NULL;
  this->regmlen = 0;
  scan = this->program + 1; // First BRANCH.
  if (OP(regnext(scan)) == END) // Only one top-level choice.
  {
    scan = OPERAND(scan);

    // Starting-point info.
    if (OP(scan) == EXACTLY)
      this->regstart = *OPERAND(scan);
    else if (OP(scan) == BOL)
      this->reganch++;

     //
     // If there's something expensive in the r.e., find the longest
     // literal string that must appear and make it the regmust.  Resolve
     // ties in favor of later strings, since the regstart check works
     // with the beginning of the r.e. and avoiding duplication
     // strengthens checking.  Not a strong reason, but sufficient in the
     // absence of others.
     //
    if (flags & SPSTART) {
      longest = NULL;
      len = 0L;
      for (; scan != NULL; scan = regnext(scan))
        if (OP(scan) == EXACTLY && std::strlen(OPERAND(scan)) >= len) {
          longest = OPERAND(scan);
          len = std::strlen(OPERAND(scan));
        }
      this->regmust = longest;
      this->regmlen = (int)len;
    }
  }
}


// regular expression, i.e. main body or parenthesized thing
//
// Caller must absorb opening parenthesis.
//
// Combining parenthesis handling with the base level of regular expression
// is a trifle forced, but the need to tie the tails of the branches to what
// follows makes it hard to avoid.
//
static char* reg (int paren, int *flagp)
{
  register char* ret;
  register char* br;
  register char* ender;
  register int   parno =0;
       int   flags;

  *flagp = HASWIDTH; // Tentatively.

  // Make an OPEN node, if parenthesized.
  if (paren) {
    if (regnpar >= reg_exp_nsubexp) {
      //RAISE Error, SYM(reg_exp), SYM(Too_Many_Parens),
      std::cout << "reg_exp::compile(): Too many parentheses.\n";
      return 0;
    }
    parno = regnpar;
    regnpar++;
    ret = regnode(OPEN + parno);
  }
  else
    ret = NULL;

  // Pick up the branches, linking them together.
  br = regbranch(&flags);
  if (br == NULL)
    return NULL;
  if (ret != NULL)
    regtail(ret, br); // OPEN -> first.
  else
    ret = br;
  if (!(flags & HASWIDTH))
    *flagp &= ~HASWIDTH;
  *flagp |= flags & SPSTART;
  while (*regparse == '|')
  {
    regparse++;
    br = regbranch(&flags);
    if (br == NULL)
      return NULL;
    regtail(ret, br); // BRANCH -> BRANCH.
    if (!(flags & HASWIDTH))
      *flagp &= ~HASWIDTH;
    *flagp |= flags & SPSTART;
  }

  // Make a closing node, and hook it on the end.
  ender = regnode((paren) ? CLOSE + parno : END);
  regtail(ret, ender);

  // Hook the tails of the branches to the closing node.
  for (br = ret; br != NULL; br = regnext(br))
    regoptail(br, ender);

  // Check for proper termination.
  if (paren && *regparse++ != ')') {
    //RAISE Error, SYM(reg_exp), SYM(Unmatched_Parens),
    std::cout << "reg_exp::compile(): Unmatched parentheses.\n";
    return 0;
  }
  else if (!paren && *regparse != '\0') {
    if (*regparse == ')') {
      //RAISE Error, SYM(reg_exp), SYM(Unmatched_Parens),
      std::cout << "reg_exp::compile(): Unmatched parentheses.\n";
      return 0;
    }
    else {
      //RAISE Error, SYM(reg_exp), SYM(Internal_Error),
      std::cout << "reg_exp::compile(): Internal error.\n";
      return 0;
    }
    // NOTREACHED
  }
  return ret;
}


// one alternative of an | operator
//
// Implements the concatenation operator.
//
static char* regbranch (int *flagp)
{
  register char* ret;
  register char* chain;
  register char* latest;
  int            flags;

  *flagp = WORST; // Tentatively.

  ret = regnode(BRANCH);
  chain = NULL;
  while (*regparse != '\0' && *regparse != '|' && *regparse != ')')
  {
    latest = regpiece(&flags);
    if (latest == NULL)
      return NULL;
    *flagp |= flags & HASWIDTH;
    if (chain == NULL) // First piece.
      *flagp |= flags & SPSTART;
    else
      regtail(chain, latest);
    chain = latest;
  }
  if (chain == NULL) // Loop ran zero times.
    regnode(NOTHING);

  return ret;
}


//
// something followed by possible [*+?]
//
// Note that the branching code sequences used for ? and the general cases
// of * and + are somewhat optimized:  they use the same NOTHING node as
// both the endmarker for their branch list and the body of the last branch.
// It might seem that this node could be dispensed with entirely, but the
// endmarker role is not redundant.
//
static char* regpiece (int *flagp)
{
  register char* ret;
  register char  op;
  register char* next;
  int            flags;

  ret = regatom(&flags);
  if (ret == NULL)
    return NULL;

  op = *regparse;
  if (!ISMULT(op)) {
    *flagp = flags;
    return ret;
  }

  if (!(flags & HASWIDTH) && op != '?') {
    //RAISE Error, SYM(reg_exp), SYM(Empty_Operand),
    std::cout << "reg_exp::compile() : *+ operand could be empty.\n";
    return 0;
  }
  *flagp = (op != '+') ? (WORST | SPSTART) : (WORST | HASWIDTH);

  if (op == '*' && (flags & SIMPLE))
    reginsert(STAR, ret);
  else if (op == '*') {
    // Emit x* as (x&|), where & means "self".
    reginsert(BRANCH, ret); // Either x
    regoptail(ret, regnode(BACK)); // and loop
    regoptail(ret, ret); // back
    regtail(ret, regnode(BRANCH)); // or
    regtail(ret, regnode(NOTHING)); // null.
  }
  else if (op == '+' && (flags & SIMPLE))
    reginsert(PLUS, ret);
  else if (op == '+') {
    // Emit x+ as x(&|), where & means "self".
    next = regnode(BRANCH); // Either
    regtail(ret, next);
    regtail(regnode(BACK), ret); // loop back
    regtail(next, regnode(BRANCH)); // or
    regtail(ret, regnode(NOTHING)); // null.
  }
  else if (op == '?') {
    // Emit x? as (x|)
    reginsert(BRANCH, ret); // Either x
    regtail(ret, regnode(BRANCH)); // or
    next = regnode(NOTHING);// null.
    regtail(ret, next);
    regoptail(ret, next);
  }
  regparse++;
  if (ISMULT(*regparse)) {
    //RAISE Error, SYM(reg_exp), SYM(Nested_Operand),
    std::cout << "reg_exp::compile(): Nested *?+.\n";
    return 0;
  }
  return ret;
}


// the lowest level
//
// Optimization:  gobbles an entire sequence of ordinary characters so that
// it can turn them into a single node, which is smaller to store and
// faster to run.  Backslashed characters are exceptions, each becoming a
// separate node; the code is simpler that way and it's not worth fixing.
//
static char* regatom (int *flagp)
{
  register char* ret;
       int   flags;

  *flagp = WORST; // Tentatively.

  switch (*regparse++)
  {
   case '^':
    ret = regnode(BOL);
    break;
   case '$':
    ret = regnode(EOL);
    break;
   case '.':
    ret = regnode(ANY);
    *flagp |= HASWIDTH | SIMPLE;
    break;
   case '[':
   {
    register int  rxpclass;
    register int  rxpclassend;

    if (*regparse == '^') { // Complement of range.
      ret = regnode(ANYBUT);
      regparse++;
    }
    else
      ret = regnode(ANYOF);
    if (*regparse == ']' || *regparse == '-')
      regc(*regparse++);
    while (*regparse != '\0' && *regparse != ']')
    {
      if (*regparse == '-')
      {
        regparse++;
        if (*regparse == ']' || *regparse == '\0')
          regc('-');
        else {
          rxpclass = UCHARAT(regparse - 2) + 1;
          rxpclassend = UCHARAT(regparse);
          if (rxpclass > rxpclassend + 1) {
             //RAISE Error, SYM(reg_exp), SYM(Invalid_Range),
             std::cout << "reg_exp::compile(): Invalid range in [].\n";
             return 0;
          }
          for (; rxpclass <= rxpclassend; rxpclass++)
            regc(rxpclass);
          regparse++;
        }
      }
      else
        regc(*regparse++);
    }
    regc('\0');
    if (*regparse != ']') {
      //RAISE Error, SYM(reg_exp), SYM(Unmatched_Bracket),
      std::cout << "reg_exp::compile(): Unmatched [].\n";
      return 0;
    }
    regparse++;
    *flagp |= HASWIDTH | SIMPLE;
    break;
   }
   case '(':
    ret = reg(1, &flags);
    if (ret == NULL)
      return NULL;
    *flagp |= flags & (HASWIDTH | SPSTART);
    break;
   case '\0':
   case '|':
   case ')':
    //RAISE Error, SYM(reg_exp), SYM(Internal_Error),
    std::cout << "reg_exp::compile(): Internal error.\n"; // Never here
    return 0;
   case '?':
   case '+':
   case '*':
    //RAISE Error, SYM(reg_exp), SYM(No_Operand),
    std::cout << "reg_exp::compile(): ?+* follows nothing.\n";
    return 0;
   case '\\':
    if (*regparse == '\0') {
      //RAISE Error, SYM(reg_exp), SYM(Trailing_Backslash),
      std::cout << "reg_exp::compile(): Trailing backslash.\n";
      return 0;
    }
    ret = regnode(EXACTLY);
    regc(*regparse++);
    regc('\0');
    *flagp |= HASWIDTH | SIMPLE;
    break;
   default:
   {
    register int  len;
    register char   ender;

    regparse--;
    len = std::strcspn(regparse, META);
    if (len <= 0) {
      //RAISE Error, SYM(reg_exp), SYM(Internal_Error),
      std::cout << "reg_exp::compile(): Internal error.\n";
      return 0;
    }
    ender = *(regparse + len);
    if (len > 1 && ISMULT(ender))
      len--; // Back off clear of ?+* operand.
    *flagp |= HASWIDTH;
    if (len == 1)
      *flagp |= SIMPLE;
    ret = regnode(EXACTLY);
    while (len > 0) {
      regc(*regparse++);
      len--;
    }
    regc('\0');
    break;
   }
  }
  return ret;
}


// emit a node
// Location.
//
static char* regnode (char op)
{
  register char* ret;
  register char* ptr;

  ret = regcode;
  if (ret == &regdummy) {
    regsize += 3;
    return ret;
  }

  ptr = ret;
  *ptr++ = op;
  *ptr++ = '\0'; // Null "next" pointer.
  *ptr++ = '\0';
  regcode = ptr;

  return ret;
}


// emit (if appropriate) a byte of code
//
static void regc (unsigned char b)
{
  if (regcode != &regdummy)
    *regcode++ = b;
  else
    regsize++;
}


// insert an operator in front of already-emitted operand
//
// Means relocating the operand.
//
static void reginsert (char op, char* opnd)
{
  register char* src;
  register char* dst;
  register char* place;

  if (regcode == &regdummy) {
    regsize += 3;
    return;
  }

  src = regcode;
  regcode += 3;
  dst = regcode;
  while (src > opnd)
    *--dst = *--src;

  place = opnd; // Op node, where operand used to be.
  *place++ = op;
  *place++ = '\0';
  *place   = '\0';
}


// set the next-pointer at the end of a node chain
//
static void regtail (char* p, const char* val)
{
  register char* scan;
  register char* temp;
  register std::ptrdiff_t  offset;

  if (p == &regdummy)
    return;

  // Find last node.
  scan = p;
  for (;;) {
    temp = regnext(scan);
    if (temp == NULL)
      break;
    scan = temp;
  }

  if (OP(scan) == BACK)
    offset = (const char*)scan - val;
  else
    offset = val - scan;
  *(scan + 1) = (char)((offset >> 8) & 0377);
  *(scan + 2) = (char)(offset & 0377);
}


// regtail on operand of first argument; nop if operandless
//
static void regoptail (char* p, const char* val)
{
  // "Operandless" and "op != BRANCH" are synonymous in practice.
  if (p == NULL || p == &regdummy || OP(p) != BRANCH)
    return;
  regtail(OPERAND(p), val);
}


////////////////////////////////////////////////////////////////////////
//
//  find and friends
//
////////////////////////////////////////////////////////////////////////


//
// Global work variables for find().
//
static const char*  reginput; // QString-input pointer.
static const char*  regbol; // Beginning of input, for ^ check.
static const char* *regstartp; // Pointer to startp array.
static const char* *regendp; // Ditto for endp.

//
// Forwards.
//
static int regtry (const char*, const char* *, const char* *, const char*);
static int regmatch (const char*);
static int regrepeat (const char*);

#ifdef DEBUG
int      regnarrate = 0;
void     regdump ();
static char* regprop ();
#endif

bool reg_exp::find (std::string const& s)
{
  return find(s.c_str());
}

//: Matches the regular expression to the given string.
// Returns true if found, and sets start and end indexes accordingly.

bool reg_exp::find (char const* string)
{
  register const char* s;

  this->searchstring = string;

   // Check validity of program.
  if (!this->program || UCHARAT(this->program) != MAGIC) {
    //RAISE Error, SYM(reg_exp), SYM(Internal_Error),
    std::cout << "reg_exp::find(): Compiled regular expression corrupted.\n";
    return 0;
  }

  // If there is a "must appear" string, look for it.
  if (this->regmust != NULL)
  {
    s = string;
    while ((s = std::strchr(s, this->regmust[0])) != NULL) {
      if (std::strncmp(s, this->regmust, this->regmlen) == 0)
        break; // Found it.
      s++;
    }
    if (s == NULL) // Not present.
      return 0;
  }

  // Mark beginning of line for ^ .
  regbol = string;

  // Simplest case:  anchored match need be tried only once.
  if (this->reganch)
    return regtry(string, this->startp, this->endp, this->program) != 0;

  // Messy cases:  unanchored match.
  s = string;
  if (this->regstart != '\0')
    // We know what char it must start with.
    while ((s = std::strchr(s, this->regstart)) != NULL) {
      if (regtry(s, this->startp, this->endp, this->program))
        return 1;
      s++;
    }
  else
    // We don't - general case.
    do {
      if (regtry(s, this->startp, this->endp, this->program))
        return 1;
    } while (*s++ != '\0');

  // Failure.
  return 0;
}


// try match at specific point
// 0 failure, 1 success
//
static int regtry(const char* string, const char* *start,
                  const char* *end, const char* prog)
{
  register       int   i;
  register const char* *sp1;
  register const char* *ep;

  reginput = string;
  regstartp = start;
  regendp = end;

  sp1 = start;
  ep = end;
  for (i = reg_exp_nsubexp; i > 0; i--) {
    *sp1++ = NULL;
    *ep++ = NULL;
  }
  if (regmatch(prog + 1)) {
    start[0] = string;
    end[0] = reginput;
    return 1;
  }
  else
    return 0;
}


// main matching routine
//
// Conceptually the strategy is simple:  check to see whether the current
// node matches, call self recursively to see whether the rest matches,
// and then act accordingly.  In practice we make some effort to avoid
// recursion, in particular by going through "ordinary" nodes (that don't
// need to know whether the rest of the match failed) by a loop instead of
// by recursion.
// 0 failure, 1 success
//
static int regmatch(const char* prog)
{
  register const char* scan; // Current node.
  const char* next; // Next node.

  scan = prog;

  while (scan != NULL)
  {
    next = regnext(scan);

    switch (OP(scan))
    {
     case BOL:
      if (reginput != regbol)
        return 0;
      break;
     case EOL:
      if (*reginput != '\0')
        return 0;
      break;
     case ANY:
      if (*reginput == '\0')
        return 0;
      reginput++;
      break;
     case EXACTLY:
     {
      register int     len;
      register const char* opnd;

      opnd = OPERAND(scan);
      // Inline the first character, for speed.
      if (*opnd != *reginput)
        return 0;
      len = std::strlen(opnd);
      if (len > 1 && std::strncmp(opnd, reginput, len) != 0)
        return 0;
      reginput += len;
      break;
     }
     case ANYOF:
      if (*reginput == '\0' || std::strchr(OPERAND(scan), *reginput) == NULL)
        return 0;
      reginput++;
      break;
     case ANYBUT:
      if (*reginput == '\0' || std::strchr(OPERAND(scan), *reginput) != NULL)
        return 0;
      reginput++;
      break;
     case NOTHING:
      break;
     case BACK:
      break;
     case OPEN+1: case OPEN+2: case OPEN+3: case OPEN+4: case OPEN+5: case OPEN+6: case OPEN+7: case OPEN+8: case OPEN+9:
     {
      register int no;
      register const char* save;

      no = OP(scan) - OPEN;
      save = reginput;

      if (regmatch(next))
      {
        //
        // Don't set startp if some later invocation of the
        // same parentheses already has.
        //
        if (regstartp[no] == NULL)
          regstartp[no] = save;
        return 1;
      }
      else
        return 0;
     }
     case CLOSE+1: case CLOSE+2: case CLOSE+3: case CLOSE+4: case CLOSE+5: case CLOSE+6: case CLOSE+7: case CLOSE+8: case CLOSE+9:
     {
      register     int  no;
      register const char* save;

      no = OP(scan) - CLOSE;
      save = reginput;

      if (regmatch(next))
      {
        //
        // Don't set endp if some later invocation of the
        // same parentheses already has.
        //
        if (regendp[no] == NULL)
          regendp[no] = save;
        return 1;
      }
      else
        return 0;
     }
     case BRANCH:
     {
      register const char* save;

      if (OP(next) != BRANCH) // No choice.
        next = OPERAND(scan); // Avoid recursion.
      else {
        do {
          save = reginput;
          if (regmatch(OPERAND(scan)))
            return 1;
          reginput = save;
          scan = regnext(scan);
        } while (scan != NULL && OP(scan) == BRANCH);
        return 0;
        // NOTREACHED
      }
      break;
     }
     case STAR:
     case PLUS:
     {
      register char   nextch;
      register int    no;
      register const char* save;
      register int    min_no;

      //
      // Lookahead to avoid useless match attempts when we know
      // what character comes next.
      //
      nextch = '\0';
      if (OP(next) == EXACTLY)
        nextch = *OPERAND(next);
      min_no = (OP(scan) == STAR) ? 0 : 1;
      save = reginput;
      no = regrepeat(OPERAND(scan));
      while (no >= min_no) {
        // If it could work, try it.
        if (nextch == '\0' || *reginput == nextch)
          if (regmatch(next))
            return 1;
        // Couldn't or didn't - back up.
        no--;
        reginput = save + no;
      }
      return 0;
     }
     case END:
      return 1; // Success!

     default:
      //RAISE Error, SYM(reg_exp), SYM(Internal_Error),
      std::cout << "reg_exp::find(): Internal error -- memory corrupted.\n";
      return 0;
    }
    scan = next;
  }

  //
  //  We get here only if there's trouble - normally "case END" is the
  //  terminating point.
  //
  //RAISE Error, SYM(reg_exp), SYM(Internal_Error),
  std::cout << "reg_exp::find(): Internal error -- corrupted pointers.\n";
  return 0;
}


// repeatedly match something simple, report how many
//
static int regrepeat(const char* p)
{
  register     int     count = 0;
  register const char* scan;
  register const char* opnd;

  scan = reginput;
  opnd = OPERAND(p);
  switch (OP(p))
  {
   case ANY:
    count = std::strlen(scan);
    scan += count;
    break;
   case EXACTLY:
    while (*opnd == *scan) {
      count++;
      scan++;
    }
    break;
   case ANYOF:
    while (*scan != '\0' && std::strchr(opnd, *scan) != NULL) {
      count++;
      scan++;
    }
    break;
   case ANYBUT:
    while (*scan != '\0' && std::strchr(opnd, *scan) == NULL) {
      count++;
      scan++;
    }
    break;
   default: // Oh dear.  Called inappropriately.
    //RAISE Error, SYM(reg_exp), SYM(Internal_Error),
    std::cout << "reg_exp::find(): Internal error.\n";
    return 0;
  }
  reginput = scan;
  return count;
}


// dig the "next" pointer out of a node
//
static const char* regnext(register const char* p)
{
  register int offset;

  if (p == &regdummy)
    return NULL;

  offset = NEXT(p);
  if (offset == 0)
    return NULL;

  if (OP(p) == BACK)
    return p - offset;
  else
    return p + offset;
}


static char* regnext(register char* p)
{
  register int offset;

  if (p == &regdummy)
    return NULL;

  offset = NEXT(p);
  if (offset == 0)
    return NULL;

  if (OP(p) == BACK)
    return p - offset;
  else
    return p + offset;
}

} // end of ntk
