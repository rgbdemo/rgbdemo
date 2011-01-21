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

#ifndef NTK_IMAGE_SIFT_GPU_H
# define NTK_IMAGE_SIFT_GPU_H

#include <ntk/core.h>

#include "siftgpu/SiftGPU.h"

#include <QLocalServer>
#include <QLocalSocket>

namespace ntk
{

  SiftGPU* getSiftGPUInstance();

  class GPUSiftDetector
  {
  public:
    int descriptorSize() const { return 128; }

    void operator()(const cv::Mat1b& img, const cv::Mat& mask,
                    std::vector<cv::KeyPoint>& keypoints,
                    std::vector<float>& descriptors) const;
  };

  class GPUSiftServer
  {
  public:
    void run();
    void stop();
    static bool isSupported();
    int descriptorSize() const { return 128; }

  private:
    void receiveRequest();
    void detectKeypoints();
    void sendPoints();
    void quit();

  private:
    QLocalServer m_server;
    QLocalSocket* m_current_socket;
    cv::Mat1b m_current_image;
    std::vector<cv::KeyPoint> m_current_keypoints;
    std::vector<float> m_current_descriptors;
    GPUSiftDetector m_detector;
    int m_pid;
  };

  class GPUSiftClient
  {
  public:
    int descriptorSize() const { return 128; }

    void operator()(const cv::Mat1b& img, const cv::Mat& mask,
                    std::vector<cv::KeyPoint>& keypoints,
                    std::vector<float>& descriptors) const;
  };

} // ntk

#endif // !NTK_IMAGE_SIFT_GPU_H
