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

#include "color_model.h"
#include <ntk/utils/debug.h>
#include <opencv/highgui.h>

using namespace cv;

void HSColorModel :: build(const cv::Mat3b& model_image, const cv::Mat1b& mask)
{
  cv::Mat3f hsv;
  cvtColor( Mat3f(model_image), hsv, CV_BGR2HSV );

  const int hbins = 16, sbins = 16;
  int hist_size[] = {hbins, sbins};
  float h_range[] = { 0, 360 }; // hue varies from 0 (~0°red) to 180 (~360°red again)
  float s_range[] = { 0, 1 }; // saturation varies from 0 (black-gray-white) to 255
  const float* ranges[] = { h_range, s_range };
  // we compute the histogram from the 0-th and 1-st channels
  int channels[] = {0, 1};

  calcHist(&hsv, 1, channels, mask, m_histogram, 2, hist_size, ranges, true, false);

  ntk_assert(m_histogram.data, "Could not create histogram.");

  // normalize histogram
  Scalar norm = sum(m_histogram);
  m_histogram *= 1.0 / norm[0];
}

void HSColorModel :: show() const
{
  ntk_assert(m_histogram.data, "Model was not build.");
  ntk_assert(m_histogram.dims == 2, "This is for 2d histograms");

  int scale = 10;
  int x_bins = m_histogram.size[0];
  int y_bins = m_histogram.size[1];

  double min_value = 0, max_value = 0;
  minMaxLoc(m_histogram, &min_value, &max_value, 0, 0 );

  Mat3b hist_img (x_bins*scale,y_bins*scale);
  hist_img = Vec3b(0,0,0);

  for(int h = 0; h < x_bins; h++ )
  {
    for(int s = 0; s < y_bins; s++ )
    {
      float bin_val = m_histogram(h,s);
      int intensity = cvRound(bin_val*255.0/max_value);
      rectangle(hist_img,
                Point( h*scale, s*scale ),
                Point( (h+1)*scale - 1, (s+1)*scale - 1),
                CV_RGB(intensity,intensity,intensity),
                CV_FILLED );
    }
  }

  namedWindow("H-S Histogram", 0 );
  imshow("H-S Histogram", hist_img);
}

double HSColorModel :: likelihood(int h_value, int s_value) const
{
  return m_histogram(h_value, s_value);
}

// Compute likelihood image using opencv back projection.
void HSColorModel :: backProject(const cv::Mat3b& bgr_image, cv::Mat1f& likelihood_image) const
{
  ntk_assert(m_histogram.data, "Invalid histogram.");

  cv::Mat3f hsv;
  cv::Mat3f float_bgr;
  cvtColor(Mat3f(bgr_image), hsv, CV_BGR2HSV );

  float h_range[] = { 0, 360 }; // hue varies from 0 (~0°red) to 180 (~360°red again)
  float s_range[] = { 0, 1 }; // saturation varies from 0 (black-gray-white) to 255
  const float* ranges[] = { h_range, s_range };
  // we compute the histogram from the 0-th and 1-st channels
  int channels[] = {0, 1};

  calcBackProject(&hsv, 1, channels, m_histogram, likelihood_image, ranges, 1, true);
}
