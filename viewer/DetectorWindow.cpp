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

#include "DetectorWindow.h"
#include "ui_DetectorWindow.h"

#include "GuiController.h"
#include "ObjectDetector.h"

DetectorWindow::DetectorWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::DetectorWindow),
    m_controller(controller)
{
    ui->setupUi(this);
}

DetectorWindow::~DetectorWindow()
{
    delete ui;
}

void DetectorWindow :: update(const ntk::RGBDImage& image)
{
  cv::Mat1b mask; std::list<cv::Rect> rects;
  m_controller.objectDetector()->detect(image.depth(), mask, rects);
  ui->maskImage->setImage(mask);
  ui->maskImage->setRects(rects);
}

void DetectorWindow::setThresholdValues(int min_value, int max_value)
{
  ui->minThresholdSlider->setValue(min_value);
  ui->maxThresholdSlider->setValue(max_value);
  updateThreshold();
}

void DetectorWindow::updateThreshold()
{
  double min_meters = ui->minThresholdSlider->value()/100.0;
  double max_meters = ui->maxThresholdSlider->value()/100.0;

  ui->minThresholdLabel->setText(QString("%1 m").arg(min_meters, 0, 'f', 2));
  ui->maxThresholdLabel->setText(QString("%1 m").arg(max_meters, 0, 'f', 2));

  m_controller.objectDetector()->setThresholds(min_meters,
                                               max_meters);
}

void DetectorWindow::on_minThresholdSlider_valueChanged(int value)
{
  updateThreshold();
}

void DetectorWindow::on_maxThresholdSlider_valueChanged(int value)
{
  updateThreshold();
}
