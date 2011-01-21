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

#ifndef FILE_GRABBER_H
#define FILE_GRABBER_H

#include "rgbd_grabber.h"

#include <QDir>
#include <QStringList>

namespace ntk
{

class FileGrabber : public RGBDGrabber
{
public:
    FileGrabber(const std::string& path, bool is_directory);

protected:
    virtual void run();

private:
    QDir m_path;
    QStringList m_image_list;
    RGBDImage m_buffer_image;
    int m_current_image_index;
    bool m_is_directory;
};

} // ntk

#endif // FILE_GRABBER_H
