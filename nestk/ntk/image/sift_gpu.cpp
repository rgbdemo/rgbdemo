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

#include "sift_gpu.h"
#include <ntk/utils/debug.h>
#include <ntk/utils/qt_utils.h>

#ifndef _WIN32
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

using namespace cv;

namespace ntk
{

SiftGPU* getSiftGPUInstance()
{
  static SiftGPU* sift_gpu = 0;
  if (!sift_gpu)
  {
    sift_gpu = new SiftGPU();
    const char * argv[] = {"-fo", "-1",  "-v", "0"};
    int argc = sizeof(argv)/sizeof(char*);
    sift_gpu->ParseParam(argc, argv);
    if ((sift_gpu->CreateContextGL() == 0) || (sift_gpu->IsFullSupported() == 0))
    {
      delete sift_gpu;
      sift_gpu = 0;
    }
  }
  return sift_gpu;
}

void GPUSiftDetector::operator ()(const Mat1b &img,
                                  const Mat &mask,
                                  std::vector<KeyPoint> &keypoints,
                                  std::vector<float> &descriptors) const
{
  SiftGPU* sift = getSiftGPUInstance();
  ntk_ensure(sift, "Cannot use SiftGPU");

  std::vector<SiftGPU::SiftKeypoint> gpu_keypoints;

  bool ok = sift->RunSIFT(img);

  int n_features = sift->GetFeatureNum();
  gpu_keypoints.resize(n_features);
  descriptors.resize(128*n_features);
  sift->GetFeatureVector(&gpu_keypoints[0], &descriptors[0]);

  keypoints.resize(n_features);
  for (int i = 0; i < n_features; ++i)
  {
    cv::KeyPoint& location = keypoints[i];
    location.pt = Point2f(gpu_keypoints[i].x, gpu_keypoints[i].y);
    location.angle = gpu_keypoints[i].o;
    location.octave = gpu_keypoints[i].s;
    location.size = gpu_keypoints[i].s * 16;
  }
}

} // ntk

namespace ntk
{

bool GPUSiftServer::isSupported()
{
#ifdef _WIN32
  return false;
#else
  return true;
#endif
}

void GPUSiftServer::stop()
{
#ifndef _WIN32
  QLocalSocket socket;
  socket.connectToServer("nestk_sift_gpu");
  if (!socket.waitForConnected(1000))
  {
    ntk_dbg(0) << "Could not connect to GPU SIFT server!";
    return;
  }
  QDataStream stream(&socket);
  stream << qint32(-1);
  socket.waitForDisconnected();
  int pid_w = -1;
  do {
    ntk_dbg(1) << "Waiting for " << m_pid << "  to finish .. \n";
    int status = 0;
    pid_w = waitpid(m_pid,&status,0);
  } while (pid_w == -1);
#else
  fatal_error("Not supported on Window");
#endif
}

void GPUSiftServer::run()
{
#ifndef _WIN32
  m_pid = fork();
  if (m_pid == -1)
  {
    fatal_error("Could not fork");
  }
  else if (m_pid != 0)
  {
    // Parent
    return;
  }

  bool listen_ok = false;
  int trials = 0;
  while (!listen_ok && trials < 5)
  {
    listen_ok = m_server.listen("nestk_sift_gpu");
    if (!listen_ok)
    {
      ntk_dbg(0) << "Could not start server, trying to clean it";
      m_server.removeServer("nestk_sift_gpu");
    }
    ++trials;
  }
  if (!listen_ok)
  {
    ntk_dbg(0) << "[ERROR] Could not start GPU Sift server.";
    exit(1);
  }

  while (true)
  {
    bool has_connection = m_server.waitForNewConnection(1000);
    if (has_connection)
    {
      ntk_dbg(2) << "New connection!";
      m_current_socket = m_server.nextPendingConnection();
      receiveRequest();
      detectKeypoints();
      sendPoints();
      m_current_socket->waitForDisconnected();
      // m_current_socket->disconnectFromServer();
      delete m_current_socket;
    }
  }
#else
  fatal_error("Not supported on Windows");
#endif
}

void GPUSiftServer::quit()
{
  ntk_dbg(2) << "Quit signal received.";
  m_server.close();
  exit(0);
}

void GPUSiftServer::receiveRequest()
{
  QDataStream stream(m_current_socket);
  ntk_assert(stream.status() == QDataStream::Ok, "Cannot receive data!");

  m_current_socket->waitForReadyRead();

  qint32 rows, cols;
  stream >> rows;
  if (rows < 0)
    quit();
  stream >> cols;
  ntk_dbg_print(rows, 2);
  ntk_dbg_print(cols, 2);
  m_current_image.create(rows, cols);
  m_current_socket->waitForReadyRead();
  readFullRawData(*m_current_socket, stream, (char*)m_current_image.data, rows*cols);
  ntk_assert(stream.status() == QDataStream::Ok, "Bad transmission within loop");
  ntk_assert(stream.status() == QDataStream::Ok, "Bad transmission");
}

void GPUSiftServer::detectKeypoints()
{
  m_detector(m_current_image, Mat(), m_current_keypoints, m_current_descriptors);
}

void GPUSiftServer::sendPoints()
{
  QDataStream stream(m_current_socket);
  qint32 num_features = m_current_keypoints.size();
  ntk_dbg_print(num_features, 2);

  stream << num_features;
  int num_bytes = stream.writeRawData((const char*)&m_current_keypoints[0],
                                      sizeof(KeyPoint)*num_features);
  ntk_assert(num_bytes == sizeof(KeyPoint)*num_features, "Not all data written.");
  ntk_dbg(2) << num_bytes << " transmitted for keypoints";
  m_current_socket->waitForBytesWritten();

  num_bytes = stream.writeRawData((const char*)&m_current_descriptors[0],
                                  sizeof(float)*num_features*descriptorSize());
  ntk_assert(num_bytes == sizeof(float)*num_features*descriptorSize(), "Not all data written.");
  ntk_dbg(2) << num_bytes << " transmitted for descriptors";

  m_current_socket->waitForBytesWritten();

  qint32 ack = 0;
  m_current_socket->waitForReadyRead();
  stream >> ack;
  ntk_dbg_print(ack, 2);
  ntk_assert(ack == 42, "Bad ack");
}

void GPUSiftClient::operator ()(const cv::Mat1b &img,
                                const cv::Mat &mask,
                                std::vector<cv::KeyPoint> &keypoints,
                                std::vector<float> &descriptors) const
{
  QLocalSocket socket;
  socket.connectToServer("nestk_sift_gpu");
  if (!socket.waitForConnected(1000))
  {
    ntk_dbg(0) << "Could not connect to GPU SIFT server!";
    keypoints.clear();
    descriptors.clear();
    return;
  }

  // Send image
  QDataStream stream(&socket);
  ntk_dbg(2) << "Sending " << img.rows << " " << img.cols;
  stream << (qint32)img.rows << (qint32)img.cols;
  int num_bytes = stream.writeRawData((const char*)img.data, img.rows*img.cols);
  ntk_assert(num_bytes == img.rows*img.cols, "Could not send all data");

  // Read keypoints
  socket.waitForReadyRead();
  qint32 num_features = -1;
  stream >> num_features;
  ntk_dbg_print(num_features, 2);

  keypoints.resize(num_features);
  readFullRawData(socket, stream, (char*)&keypoints[0], num_features*sizeof(KeyPoint));
  ntk_assert(stream.status() == QDataStream::Ok, "Bad transmission.");

  descriptors.resize(num_features*descriptorSize());
  readFullRawData(socket, stream, (char*)&descriptors[0],
                  num_features*descriptorSize()*sizeof(float));
  ntk_assert(stream.status() == QDataStream::Ok, "Bad transmission.");

  stream << (qint32) 42;
  socket.waitForBytesWritten();
  socket.disconnectFromServer();
}

} // ntk
