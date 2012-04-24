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

#include "CalibrationWindow.h"
#include "ui_CalibrationWindow.h"
#include "GuiController.h"

#include <ntk/camera/calibration.h>

#include <QFileDialog>

using namespace ntk;

CalibrationWindow::CalibrationWindow(GuiController &controller, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CalibrationWindow),
    controller(controller)
{
    ui->setupUi(this);
}

CalibrationWindow::~CalibrationWindow()
{
    delete ui;
}

void CalibrationWindow::on_addFrameButton_clicked()
{
    if (!controller.lastImage().hasRgb())
    {
        ntk_dbg(0) << "No image with RGB available for calibration.";
        return;
    }

    int pattern_width = ui->patternWidthBox->value();
    int pattern_height = ui->patternHeightBox->value();
    float pattern_size = ui->patternSizeBox->value();

    std::vector<cv::Point2f> corners;
    cv::Mat3b checkerboard_image;
    calibrationCorners("checkboard", "",
                       pattern_width, pattern_height,
                       corners,
                       controller.lastImage().rgb(),
                       1.0,
                       PatternChessboard,
                       &checkerboard_image);
    ui->lastImageWidget->setImage(controller.lastImage().depth());
    ui->lastCheckerboardWidget->setImage(checkerboard_image);
    images.push_back(controller.lastImage());
    ui->checkerboardLabel->setText(QString("%1 images").arg(images.size()));
}

void CalibrationWindow::on_calibrateButton_clicked()
{
    int pattern_width = ui->patternWidthBox->value();
    int pattern_height = ui->patternHeightBox->value();
    float pattern_size = ui->patternSizeBox->value();

    std::vector< std::vector<cv::Point2f> > good_corners;
    std::vector< std::vector<cv::Point2f> > all_corners;
    getCalibratedCheckerboardCorners(images,
                                     pattern_width, pattern_height,
                                     PatternChessboard,
                                     all_corners, good_corners, false /* show */);

    float scale_factor_mean = calibrate_kinect_scale_factor(images, all_corners,
                                                            pattern_width, pattern_height, pattern_size);

    images[0].calibration()->copyTo(last_calibration);
    last_calibration.rgb_intrinsics(0,0) /= scale_factor_mean;
    last_calibration.rgb_intrinsics(1,1) /= scale_factor_mean;
    ntk_dbg_print(scale_factor_mean, 1);

    double width_ratio = double(last_calibration.rgbSize().width)/last_calibration.depthSize().width;
    double height_ratio = double(last_calibration.rgbSize().height)/last_calibration.depthSize().height;

    last_calibration.rgb_intrinsics.copyTo(last_calibration.depth_intrinsics);
    last_calibration.rgb_distortion.copyTo(last_calibration.depth_distortion);
    last_calibration.depth_intrinsics(0,0) /= width_ratio;
    last_calibration.depth_intrinsics(1,1) /= width_ratio;
    last_calibration.depth_intrinsics(0,2) /= width_ratio;
    last_calibration.depth_intrinsics(1,2) /= width_ratio;
    last_calibration.updatePoses();

    ui->checkerboardLabel->setText(QString("%1 images - Calibration OK - Scale factor = %2").arg(images.size()).arg(scale_factor_mean));
}

void CalibrationWindow::on_resetButton_clicked()
{
    images.clear();
    ui->checkerboardLabel->setText(QString("0 images"));
}

void CalibrationWindow::on_saveCalibrationButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    "Save calibration as...",
                                                    QString("calibration-%1.yml").arg(controller.lastImage().cameraSerial().c_str()));
    if (filename.isEmpty())
       return;
    last_calibration.saveToFile(filename.toAscii());
}
