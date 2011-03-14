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

#ifndef MODELACQUISITIONCONTROLLER_H
#define MODELACQUISITIONCONTROLLER_H

#include <ntk/core.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/geometry/relative_pose_estimator.h>
#include <ntk/camera/calibration.h>
#include <ntk/mesh/rgbd_modeler.h>

#include <QFuture>

class GuiController;

class ModelAcquisitionController
{
public:
  ModelAcquisitionController(GuiController& controller,
                             ntk::RGBDModeler& modeler)
                               : m_controller(controller),
                                 m_modeler(modeler),
                                 m_pose_estimator(0),
                                 m_paused(true)
  {}

  void setPaused(bool paused) { m_paused = paused; }
  virtual void newFrame(const ntk::RGBDImage& image);
  virtual int getNumberOfSteps() { return 35; }
  virtual void grabAndMove();
  virtual void modelAndMove();
  virtual void move() {}
  virtual void reset();
  void setPoseEstimator(ntk::RelativePoseEstimator* estimator) { m_pose_estimator = estimator; }
  ntk::RGBDModeler& modeler() { return m_modeler; }

private:
  bool newFrameThread(const ntk::RGBDImage* image);

protected:
  GuiController& m_controller;
  ntk::RGBDModeler& m_modeler;
  ntk::RelativePoseEstimator* m_pose_estimator;
  QFuture<bool> m_new_frame_run;
  ntk::RGBDImage m_current_image;
  bool m_paused;
};

#endif // MODELACQUISITIONCONTROLLER_H
