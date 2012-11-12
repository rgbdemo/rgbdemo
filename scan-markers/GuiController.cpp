/**
 * This file is part of the rgbdemo project.
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
 * Author: Nicolas Burrus <nicolas@burrus.name>, (C) 2010, 2011
 */

#include "GuiController.h"
#include "ui_RawImagesWindow.h"
#include "FiltersWindow.h"
#include "RawImagesWindow.h"
#include "ModelAcquisitionWindow.h"
#include "ModelAcquisitionController.h"

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
    m_model_acquisition_controller(0),
    m_last_tick(cv::getTickCount()),
    m_frame_counter(0),
    m_frame_rate(0),
    m_modeling_window_grabber("/tmp/demo3d/modeling"),
    m_raw_window_grabber("/tmp/demo3d/raw"),
    m_screen_capture_mode(false),
    m_grab_frames(false),
    m_paused(false),
    m_process_one_frame(false)
{
  m_raw_images_window = new RawImagesWindow(*this);
  m_filters_window = new FiltersWindow(*this);
  m_model_window = new ModelAcquisitionWindow(*this);

  m_raw_images_window->show();
  m_model_window->show();
}

GuiController :: ~GuiController()
{
  delete m_raw_images_window;
  delete m_filters_window;
  delete m_model_window;
}

void GuiController :: quit()
{
  m_grabber.stop();
  QApplication::quit();
}

void GuiController :: setModelAcquisitionController(ModelAcquisitionController& controller)
{
  m_model_acquisition_controller = &controller;
  m_raw_images_window->ui->action_Show_Modeler->setEnabled(true);
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

  //ntk_dbg_print(m_last_image.directory(), 1);

#if 0 // useful for horizontal alignment of cameras
  line(m_last_image.rgbRef(), Point(400, 0), Point(400,600), Scalar(0,0,0,255));
  line(m_last_image.amplitudeRef(), Point(102, 0), Point(102,204), Scalar(0,0,0));
#endif

  // imshow("color", m_last_image.rgb());

  if (m_raw_images_window->isVisible())
    m_raw_images_window->update(m_last_image);

  if (m_frame_recorder && (m_screen_capture_mode || m_grab_frames))
    m_frame_recorder->saveCurrentFrame(m_last_image);

  if (m_screen_capture_mode)
    m_raw_window_grabber.saveFrame(QPixmap::grabWindow(m_raw_images_window->winId()));

  if (m_model_acquisition_controller
      && m_model_window->isVisible())
  {
    m_model_acquisition_controller->newFrame(m_last_image);
    if (m_screen_capture_mode)
      m_modeling_window_grabber.saveFrame(QPixmap::grabWindow(m_model_window->winId()));
  }

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

void GuiController::toggleFilters(bool active)
{
  if (active)
  {
    m_filters_window->show();
  }
  else
  {
    m_filters_window->hide();
  }
}

void GuiController::toggleModeler(bool active)
{
  if (active)
  {
    m_model_window->show();
  }
  else
  {
    m_model_window->hide();
  }
}

