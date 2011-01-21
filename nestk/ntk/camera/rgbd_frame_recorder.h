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

#ifndef NTK_CAMERA_FRAMERECORDER_H
#define NTK_CAMERA_FRAMERECORDER_H

#include <ntk/core.h>

#include <ntk/camera/rgbd_grabber.h>

#include <QDir>

namespace ntk
{

class RGBDFrameRecorder
{
public:
  RGBDFrameRecorder(const std::string& directory);

  void saveCurrentFrame(const RGBDImage& image);
  void setSaveOnlyRaw(bool v) { m_only_raw = v; }
  void setFrameIndex(int index) { m_frame_index = index; }
  void resetFrameIndex() { m_frame_index = 0; }
  void setDirectory(const std::string& directory);
  const QDir& directory() const { return m_dir; }

private:
  QDir m_dir;
  int m_frame_index;
  bool m_only_raw;
};

} // ntk

#endif // NTK_CAMERA_FRAMERECORDER_H
