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

#include "FiltersWindow.h"
#include "ui_FiltersWindow.h"

#include "GuiController.h"
#include <ntk/camera/rgbd_grabber.h>
#include <ntk/camera/rgbd_processor.h>

using namespace ntk;

FiltersWindow::FiltersWindow(GuiController& controller, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FiltersWindow),
    m_controller(controller)
{
  ui->setupUi(this);

  if ( m_controller.rgbdProcessor().hasFilterFlag(RGBDProcessor::FilterEdges))
    ui->edgesCheckBox->setCheckState(Qt::Checked);

  if ( m_controller.rgbdProcessor().hasFilterFlag(RGBDProcessor::FilterAmplitude))
    ui->amplitudeCheckBox->setCheckState(Qt::Checked);

  if ( m_controller.rgbdProcessor().hasFilterFlag(RGBDProcessor::FilterThresholdDepth))
    ui->depthThresholdCheckBox->setCheckState(Qt::Checked);

  if ( m_controller.rgbdProcessor().hasFilterFlag(RGBDProcessor::FilterNormals))
    ui->normalsCheckBox->setCheckState(Qt::Checked);

  if ( m_controller.rgbdProcessor().hasFilterFlag(RGBDProcessor::FilterUnstable))
    ui->unstableCheckBox->setCheckState(Qt::Checked);

  if ( m_controller.rgbdProcessor().hasFilterFlag(RGBDProcessor::FilterMedian))
    ui->medianCheckBox->setCheckState(Qt::Checked);

  ui->minDepthSlider->setValue(m_controller.rgbdProcessor().minDepth()*100);
  ui->maxDepthSlider->setValue(m_controller.rgbdProcessor().maxDepth()*100);
  updateDepthSlider(); // to update labels.
}

FiltersWindow::~FiltersWindow()
{
  delete ui;
}

void FiltersWindow::on_depthThresholdCheckBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::FilterThresholdDepth, checked);
}

void FiltersWindow::on_edgesCheckBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::FilterEdges, checked);
}

void FiltersWindow::on_amplitudeCheckBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::FilterAmplitude, checked);
}

void FiltersWindow::on_medianCheckBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::FilterMedian, checked);
}

void FiltersWindow::on_normalsCheckBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::FilterNormals, checked);
  m_controller.rgbdProcessor().setMaxNormalAngle(60);
}

void FiltersWindow::on_unstableCheckBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::FilterNormals, checked);
}

void FiltersWindow::updateDepthSlider()
{
  double min_meters = ui->minDepthSlider->value()/100.0;
  double max_meters = ui->maxDepthSlider->value()/100.0;

  ui->minDepthLabel->setText(QString("%1 m").arg(min_meters, 0, 'f', 2));
  ui->maxDepthLabel->setText(QString("%1 m").arg(max_meters, 0, 'f', 2));

  m_controller.rgbdProcessor().setMinDepth(min_meters);
  m_controller.rgbdProcessor().setMaxDepth(max_meters);
}

void FiltersWindow::updateAmplitudeSlider()
{
  double min_amplitude = ui->minAmplitudeSlider->value();
  double max_amplitude = ui->maxAmplitudeSlider->value();

  ui->minAmplitudeLabel->setText(QString("%1").arg(min_amplitude, 0, 'f', 2));
  ui->maxAmplitudeLabel->setText(QString("%1").arg(max_amplitude, 0, 'f', 2));

  m_controller.rgbdProcessor().setMinAmplitude(min_amplitude);
  m_controller.rgbdProcessor().setMaxAmplitude(max_amplitude);
}

void FiltersWindow::on_minDepthSlider_valueChanged(int value)
{
  updateDepthSlider();
}

void FiltersWindow::on_maxDepthSlider_valueChanged(int value)
{
  updateDepthSlider();
}

void FiltersWindow::on_minAmplitudeSlider_valueChanged(int value)
{
  updateAmplitudeSlider();
}

void FiltersWindow::on_maxAmplitudeSlider_valueChanged(int value)
{
  updateAmplitudeSlider();
}


void FiltersWindow::on_fillSmallHolesCheckBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::FillSmallHoles, checked);
}

void FiltersWindow::on_removeSmallStructuresBox_toggled(bool checked)
{
  m_controller.rgbdProcessor().setFilterFlag(RGBDProcessor::RemoveSmallStructures, checked);
}

void FiltersWindow::on_kinectTiltSlider_valueChanged(int value)
{
  m_controller.grabber().setTiltAngle(value);
}
