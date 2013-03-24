/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#include "GuiController.h"
#include "ui_RawImagesWindow.h"
#include "RawImagesWindow.h"

#include <ntk/ntk.h>
#include <ntk/camera/rgbd_frame_recorder.h>

#include <QMainWindow>
#include <QImage>
#include <QApplication>

using namespace cv;
using namespace ntk;

GuiController :: GuiController(ntk::RGBDGrabber& producer,
                               ntk::RGBDProcessor& processor)
  : m_grabber(producer),
    m_processor(processor),
    m_frame_recorder(0),
    m_last_tick(cv::getTickCount()),
    m_frame_counter(0),
    m_frame_rate(0),
    m_raw_window_grabber("/tmp/demo3d/raw"),
    m_screen_capture_mode(false),
    m_grab_frames(false),
    m_paused(false),
    m_process_one_frame(false)
{
  m_raw_images_window = new RawImagesWindow(*this);
  m_raw_images_window->show();
}

GuiController :: ~GuiController()
{
  delete m_raw_images_window;
}

void GuiController :: quit()
{
  m_grabber.setThreadShouldExit();
  m_grabber.newEvent();
  m_grabber.wait();
  QApplication::quit();
}

void GuiController :: setFrameRecorder(ntk::RGBDFrameRecorder& frame_recorder)
{
  m_frame_recorder = &frame_recorder;
  m_raw_images_window->ui->outputDirText->setText(m_frame_recorder->directory().path());
}

void GuiController :: onRGBDDataUpdated()
{
  if (m_paused && !m_process_one_frame)
    return;

  m_process_one_frame = false;

  ++m_frame_counter;
  if (m_frame_counter == 10)
  {
    double current_tick = cv::getTickCount();
    m_frame_rate = m_frame_counter / ((current_tick - m_last_tick)/cv::getTickFrequency());
    m_last_tick = current_tick;
    m_frame_counter = 0;
  }

  m_grabber.copyImageTo(m_last_image);
  TimeCount tc_rgbd_process("m_processor.processImage", 2);
  m_processor.processImage(m_last_image);
  tc_rgbd_process.stop();

  if (m_raw_images_window->isVisible())
    m_raw_images_window->update(m_last_image);

  if (m_frame_recorder && (m_screen_capture_mode || m_grab_frames))
    m_frame_recorder->saveCurrentFrame(m_last_image);

  if (m_screen_capture_mode)
    m_raw_window_grabber.saveFrame(QPixmap::grabWindow(m_raw_images_window->winId()));

  QString status = QString("Final fps = %1 Fps GRABBER = %2")
                   .arg(m_frame_rate, 0, 'f', 1)
                   .arg(m_grabber.frameRate(), 0, 'f', 1);
  if (m_grab_frames)
    status += " [GRABBING]";
  m_raw_images_window->ui->statusbar->showMessage(status);

  if (!m_paused)
    m_grabber.newEvent();
}

void GuiController :: on_depth_mouse_moved(int x, int y)
{
  if (!m_last_image.depth().data || x < 0 || y < 0)
    return;
  QString s = QString("Distance at (%1,%2) = %3 m")
              .arg(x).arg(y).arg(m_last_image.depth()(y,x), 0, 'f', 3);
  m_raw_images_window->ui->distanceLabel->setText(s);
}

