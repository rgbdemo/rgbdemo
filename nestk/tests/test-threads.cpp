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

#include <ntk/utils/time.h>
#include <QThread>

using namespace ntk;

cv::RNG rng;

struct Functor : public QThread
{
  virtual void run()
  {
    ntk::sleep(rng(5000));
    std::cout << "hello world" << std::endl;
  }
};

int main()
{
  static const int n_threads = 100;

  Functor functors[n_threads];

  for (int i = 0; i < n_threads; ++i)
  {
    functors[i].start();
  }

  for (int i = 0; i < n_threads; ++i)
  {
    functors[i].wait();
  }
  return 0;
}
