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

#ifndef         OHC_ARG_H_
# define        OHC_ARG_H_

# include <vector>
# include <string>
# include <list>
# include <iosfwd>
# include <cstddef>

namespace ntk
{

const int reg_exp_nsubexp = 10;

//: Pattern matching with regular expressions.
//  A regular expression allows a programmer to specify complex
//  patterns that can be searched for and matched against the
//  character string of a string object. In its simplest form, a
//  regular expression is a sequence of characters used to search for
//  exact character matches. However, many times the exact sequence to
//  be found is not known, or only a match at the beginning or end of
//  a string is desired. This regular expression class implements
//  regular expression pattern matching as is found and implemented in
//  many UNIX commands and utilities.
//
//  Example: The perl code
// \code
//     $filename =~ m"([a-z]+)\.cc";
//     print $1;
// \endcode
//  is written as follows in C++
// \code
//     reg_exp re("([a-z]+)\\.cc");
//     re.find(filename);
//     std::cout << re.match(1);
// \endcode
//
//  The regular expression class provides a convenient mechanism for
//  specifying and manipulating regular expressions. The regular
//  expression object allows specification of such patterns by using
//  the following regular expression metacharacters:
//
// -  ^        Matches at beginning of a line
// -  $        Matches at end of a line
// - .         Matches any single character
// - [ ]       Matches any character(s) inside the brackets
// - [^ ]      Matches any character(s) not inside the brackets
// - [ - ]     Matches any character in range on either side of a dash
// -  *        Matches preceding pattern zero or more times
// -  +        Matches preceding pattern one or more times
// -  ?        Matches preceding pattern at most once
// - ()        Saves a matched expression and uses it in a later match
//
//  Note that more than one of these metacharacters can be used in a
//  single regular expression in order to create complex search
//  patterns. For example, the pattern [^ab1-9] says to match any
//  character sequence that does not begin with the characters "a",
//  "b", or the characters "1" through "9".
//
class reg_exp
{
  //: anchor point of start position for n-th matching regular expression
  const char* startp[reg_exp_nsubexp];
  //: anchor point of end position for n-th matching regular expression
  const char* endp[reg_exp_nsubexp];
  //: Internal use only
  char  regstart;
  //: Internal use only
  char  reganch;
  //: Internal use only
  const char* regmust;
  //: Internal use only
  int   regmlen;
  char* program;
  int   progsize;
  const char* searchstring;
 public:
  //: Creates an empty regular expression.
  inline reg_exp() : program(0) { clear_bufs(); }
  //: Creates a regular expression from string s, and compiles s.
  inline reg_exp(char const* s) : program(0) { clear_bufs(); compile(s); }
  //: Copy constructor
  reg_exp(reg_exp const&);
  //: Frees space allocated for regular expression.
  inline ~reg_exp() { delete[] this->program; }
  //: Compiles char* --> regexp
  void compile(char const*);
  //: true if regexp in char* arg
  bool find(char const*);
  //: true if regexp in char* arg
  bool find(std::string const&);
  //: Returns the start index of the last item found.
  inline std::ptrdiff_t start() const { return this->startp[0] - searchstring; }
  //: Returns the end index of the last item found.
  inline std::ptrdiff_t end()   const { return this->endp[0] - searchstring; }
  //: Equality operator
  bool operator==(reg_exp const&) const;
  //: Inequality operator
  inline bool operator!=(reg_exp const& r) const { return !operator==(r); }
  //: Same regexp and state?
  bool deep_equal(reg_exp const&) const;
  //: Returns true if a valid RE is compiled and ready for pattern matching.
  inline bool is_valid() const { return this->program != 0; }
  //: Invalidates regular expression.
  inline void set_invalid() { delete[] this->program; this->program = 0; clear_bufs(); }

  //: Return start index of nth submatch.
  // start(0) is the start of the full match.
  inline std::ptrdiff_t start(long n) const { return this->startp[n] - searchstring; }
  //: Return end index of nth submatch.
  // end(0) is the end of the full match.
  inline std::ptrdiff_t end(long n)   const { return this->endp[n] - searchstring; }
  //: Return nth submatch as a string.
  std::string match(int n) const {
    return this->endp[n] == this->startp[n] ? std::string("") :
           std::string(this->startp[n], this->endp[n] - this->startp[n]);
  }
  //: Return an expression that will match precisely c
  // The returned string is owned by the function, and
  // will be overwritten in subsequent calls.
  static const char * protect(char c);

 private:
  //: private function to clear startp[] and endp[]
  void clear_bufs() { for (int n=0; n<reg_exp_nsubexp; ++n) startp[n]=endp[n]=0; }
};

} // end of ntk

namespace ntk
{

//: forward declare all classes and their helper functions.
class arg_info_list;
template <class T> class arg;
template <class T> void settype     (arg<T> &);
template <class T> void print_value (std::ostream &, arg<T> const &);
template <class T> int  parse       (arg<T>*, char**);

//: This is the base class for the templated ntk_arg<T>s
class arg_base
{
 public:
  static void parse_deprecated(int& argc, char **& argv,
                               bool warn_about_unrecognized_arguments = true);
  static void include_deprecated(arg_info_list& l);

  static void add_to_current(arg_base* a);
  static void set_help_option( char const*str);
  static void set_help_description( char const*str);
  static void set_help_precis( char const*str);
  static void display_usage(char const* msg = 0);
  static void display_usage_and_exit(char const* msg = 0);

  friend class arg_info_list;

  char const* option();
  char const* help();

  //: Returns true if arg was set on the command line.
  bool set() const;

  virtual std::ostream& print_value(std::ostream&) = 0;

 public:   // Avoid errors on some compilers that don't follow
           // protected: directive correctly with type_

  //: Static text describing type of option (e.g. bool or double).
  char const *type_;
 protected:
  //: After parsing, true iff value was set on command line.
  bool set_;
  //: Option flag including "-" or "--".
  std::string option_;
  //: Description of argument.
  std::string help_;

  arg_base(arg_info_list& l, char const* option_string,
               char const*helpstring);
  arg_base(char const* option_string, char const*helpstring);
  virtual ~arg_base() {}

  virtual int parse(char ** argv) = 0;
};

void arg_parse(int& argc, char **& argv,
               bool warn_about_unrecognized_arguments = true);

//: Add an externally supplied list of args to the global list.
void arg_include(arg_info_list& l);

//: Print all args, and usage messages.
void arg_display_usage_and_exit(char const* msg = 0);

//: parse command-line arguments
template <class T>
class arg : public arg_base
{
 public:
  T value_;// public so we don't have to worry about templated friends.

  //: Construct an ntk_arg<T> with command-line switch and default value.
  // Command line switch \a option_string, and default value
  // \a default_value.  Add this argument to the global
  // list of arguments that ntk_arg_base::parse() uses when it eventually
  // gets the command line.
  //
  // If \a option_string is null, then the argument is assigned to the
  // first plain word in the command line (warning: this causes problems for
  // T=char *, but that just means that you have to have a help string if you
  // want a default... good)
  arg(char const* option_string = 0,
          char const* helpstring = 0,
          T default_value = T())
    : arg_base(option_string,helpstring),
      value_(default_value) { settype(); }

  //: As above, but add the arg to the list \a l, on which \c parse() can be called later.
  arg(arg_info_list & l,
          char const * option_string = 0,
          char const * helpstring = 0,
          T default_value = T())
    : arg_base(l, option_string, helpstring),
      value_(default_value) { settype(); }

  //: return the arg's current value, whether the default or the one from the command line.
  T      & operator () () { return value_; }
  T const& operator () () const { return value_; }
  //operator T& () { return value_; }

  //: returns number of args chomped, or -1 on failure.
  int parse(char ** argv) { return ntk::parse(this, argv); }

  //: print
  std::ostream& print_value(std::ostream &s) {
    ntk::print_value(s, *this);
    return s; // << flush
  }

 private:
  void settype() { ntk::settype(*this); }
};

//: a helper for ntk_arg::parse.
// Users might need it if they wish to parse several command lines.
//
class arg_info_list
{
 public:
  enum autonomy {
    subset,
    all
  };
  //: Construct an empty ntk_arg_info_list.
  arg_info_list(autonomy autonomy__ = subset)
    : help_("-?"), // default help operator!
      verbose_(false), autonomy_(autonomy__) {}

  ~arg_info_list() {}

  void add(arg_base* arg);
  void parse(int& argc, char **& argv, bool warn_about_unrecognized_arguments);
  void include(arg_info_list& l);
  void verbose(bool on) { verbose_ = on; }

  void set_help_option(char const* str);

  //: Set the (short) text used to describe the command
  void set_help_precis(char const* str) { command_precis_ = str; }

  //: Set the (possibly long) text used to document the command.
  // It is displayed at the end of the help page.
  void set_help_description(char const* str) { description_ = str; }

 public://protected:
  std::vector<arg_base*> args_;
  std::string help_;
  std::string description_;
  std::string command_precis_;
  bool verbose_;
  autonomy autonomy_;

  void display_help( char const* progname= 0);

 private:
  arg_info_list(arg_info_list const &) {}
  void operator=(arg_info_list const &) {}
};

} // end of ntk

#endif // OHC_ARG_H_
