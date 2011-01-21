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

#ifndef NTK_UTILS_stl_H
# define NTK_UTILS_stl_H

# include <ntk/core.h>
# include <ntk/utils/debug.h>

# include <set>
# include <algorithm>
# include <iterator>
# include <sstream>

# include <QFileInfo>

template <class T>
std::set<T> operator+(const std::set<T>& lhs, const std::set<T>& rhs)
{
  std::set<T> result;
  set_union(stl_bounds(lhs), stl_bounds(rhs), 
            std::inserter(result, result.begin()));
  return result;
}

template <class T>
std::set<T>& operator+=(std::set<T>& lhs, const std::set<T>& rhs)
{
  lhs.insert(stl_bounds(rhs));
  return lhs;
}

namespace ntk {

/*!
 * Find the maximal value in a container.
 * \param max_value Will be set to the alue of the maximal element.
 * \return Index of the maximal element, -1 if empty container.
 */
template <typename Iterator, typename ValueType>
int find_max(Iterator begin, Iterator end, ValueType& max_value)
{
  int best_index = -1;
  for (int i = 0; begin != end; ++begin, ++i)
  {
    if (best_index < 0 || *begin > max_value)
    {
      max_value = *begin;
      best_index = i;
    }
  }
  return best_index;
}

inline void draw_k_different_numbers(cv::RNG& rgen, std::set<int>& indices, int k, int n)
{
  indices.clear();
  for (int i = 0; i < k; ++i)
  {
    int index;
    do
    {
      index = rgen(n);
    } while (indices.find(index) != indices.end());
    indices.insert(index);
  }
}

template <class T>
std::string
atos(const T& t)
{
  std::ostringstream os;
  os << t;
  return os.str();
}

inline bool is_file(const std::string& filename)
{
  return QFileInfo(filename.c_str()).isFile();
}

inline bool is_directory(const std::string& filename)
{
  return QFileInfo(filename.c_str()).isDir();
}

}

namespace ntk
{

template <class ContainerType>
bool has_key(const ContainerType& container,
             const typename ContainerType::key_type& key)
{
  return container.find(key) != container.end();
}

template <class Container>
class AssociativeContainerTransaction
{
public:
  typedef typename Container::mapped_type mapped_type;
  typedef typename Container::key_type key_type;

public:
  AssociativeContainerTransaction(Container& container)
    : m_container(container)
  {
  }

  ~AssociativeContainerTransaction()
  { validateTransaction(); }

  const mapped_type& operator[] (const key_type& key)
  {
    typename Container::const_iterator it = m_container.find(key);
    ntk_ensure(it != m_container.end(), "Inexistant key.");
    return it->second;
  }

  mapped_type& changeElement(const key_type& key)
  {
    typename Container::iterator it = m_container.find(key);
    ntk_ensure(it != m_container.end(), "Inexistant key.");
    if (!has_key(m_old_values, it->first))
      m_old_values[it->first] = it->second;
    return it->second;
  }

  void removeElement(const key_type& key)
  {
    typename Container::iterator it = m_container.find(key);
    ntk_ensure(it != m_container.end(), "Inexistant key.");
    if (!has_key(m_old_values, it->first))
      m_old_values[it->first] = it->second;
    m_container.erase(key);
  }

  void abortTransaction()
  {
    for(typename Container::const_iterator it = m_old_values.begin();
        it != m_old_values.end();
        ++it)
      m_container[it->first] = it->second;
  }

  void validateTransaction()
  {
    m_old_values.clear();
  }

private:
  Container& m_container;
  Container m_old_values;
};

template <class Container>
AssociativeContainerTransaction<Container>
create_associative_container_transaction(Container& container)
{
  return AssociativeContainerTransaction<Container>(container);
}

template <class Iterator>
class key_iterator : public Iterator
{
  typedef typename std::iterator_traits<Iterator>::value_type value_type;

public:
  key_iterator(const Iterator& it)
    : Iterator(it)
  {}

  const typename value_type::first_type& operator*() const
  { return (*(Iterator*)this)->first; }

  const typename value_type::first_type* operator->() const
  { return &(*(Iterator*)this)->first; }
};

template <class Iterator>
class value_iterator : public Iterator
{
  typedef typename std::iterator_traits<Iterator>::value_type value_type;

public:
  value_iterator(const Iterator& it)
    : Iterator(it)
  {}

  const typename value_type::second_type& operator*() const
  { return (*(Iterator*)this)->second; }

  const typename value_type::second_type* operator->() const
  { return &(*(Iterator*)this)->second; }
};

} // end of ntk

namespace ntk
{

/**
   *  Filter a set of SORTED objects. The match with the lowest score
   *  must be the first one. Then for each object, if there is another
   *  object with a higher score for which predicate is true,
   *  the first object is removed.
   */
template <typename outputT, typename inputT, typename U>
void keep_maximal_among_sorted_objects(outputT output,
                                       inputT begin, inputT end,
                                       U predicate)
{
  while (begin != end)
  {
    inputT tmp = begin; ++tmp;
    bool is_locally_maximal = true;
    while (tmp != end && is_locally_maximal)
    {
      if (predicate(*begin, *tmp))
      {
        is_locally_maximal = false;
      }
      ++tmp;
    }
    if (is_locally_maximal)
    {
      *output = *begin;
      ++output;
    }
    ++begin;
  }
}

} // end of ntk

#endif // ndef NTK_UTILS_stl_H
