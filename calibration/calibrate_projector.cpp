/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 * Copyright (C) 2012 Mariano Tepper <mtepper@dc.uba.ar>
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
 * Author: Mariano Tepper <mtepper@dc.uba.ar>
 */


#include "calibration_common.h"

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>
#include <ntk/projector/calibration.h>
#include <ntk/geometry/pose_3d.h>
//#include <opencv/cv.h>
#include <fstream>

#include <QDir>
#include <QDebug>

using namespace ntk;


class Context {

public:

    Context() :
        opt_image_directory(0, "RGBD images directory", 0),
        opt_input_file("--kinect", "Input YAML filename", "kinect_calibration.yml"),
        opt_output_file("--output", "Output YAML filename", "projector_calibration.yml"),
        opt_projector_width("--projector-width", "Projector width in pixels", 1280),
        opt_projector_height("--projector-height", "Projector height in pixels", 800),
        opt_pattern_width("--pattern-width", "Pattern width (number of inner corners)", 7),
        opt_pattern_height("--pattern-height", "Pattern height (number of inner corners)", 10),
        opt_pattern_pixels("--pattern-pixels", "Pattern side length (in pixels)", 40),
        opt_square_size("--pattern-size", "Square size in used defined scale", 0.04),
        opt_offsetX_3x3("--offsetX_3x3", "Horizontal position of the 3x3 pattern", 0.66),
        opt_offsetY_3x3("--offsetY_3x3", "Vertical position of the 3x3 pattern", 0.),
        opt_offsetX_3x4("--offsetX_3x4", "Horizontal position of the 3x4 pattern", 0.),
        opt_offsetY_3x4("--offsetY_3x4", "Vertical position of the 3x4 pattern", 0.),
        opt_offsetX_3x5("--offsetX_3x5", "Horizontal position of the 3x5 pattern", 0.),
        opt_offsetY_3x5("--offsetY_3x5", "Vertical position of the 3x5 pattern", 0.33),
        opt_offsetX_3x6("--offsetX_3x6", "Horizontal position of the 3x6 pattern", 0.66),
        opt_offsetY_3x6("--offsetY_3x6", "Vertical position of the 3x6 pattern", 0.29),
	image_size(640,480),
        rgb_pattern_size_3x3(3, 3),
        rgb_pattern_size_3x4(3, 4),
        rgb_pattern_size_3x5(3, 5),
        rgb_pattern_size_3x6(3, 6)
    {}

	ntk::arg<const char*> opt_image_directory;;
	ntk::arg<const char*> opt_input_file;
	ntk::arg<const char*> opt_output_file;
	ntk::arg<int> opt_projector_width;
	ntk::arg<int> opt_projector_height;
	ntk::arg<int> opt_pattern_width;
	ntk::arg<int> opt_pattern_height;
	ntk::arg<int> opt_pattern_pixels;
	ntk::arg<float> opt_square_size;
	ntk::arg<float> opt_offsetX_3x3;
	ntk::arg<float> opt_offsetY_3x3;
	ntk::arg<float> opt_offsetX_3x4;
	ntk::arg<float> opt_offsetY_3x4;
	ntk::arg<float> opt_offsetX_3x5;
	ntk::arg<float> opt_offsetY_3x5;
	ntk::arg<float> opt_offsetX_3x6;
	ntk::arg<float> opt_offsetY_3x6;

	const cv::Size image_size;
	cv::Size proj_size;
	cv::Size proj_pattern_size;
	const cv::Size rgb_pattern_size_3x3;
	const cv::Size rgb_pattern_size_3x4;
	const cv::Size rgb_pattern_size_3x5;
	const cv::Size rgb_pattern_size_3x6;
	

	QDir images_dir;
	QStringList images_list;

	cv::Mat1d rgb_intrinsics;
	cv::Mat1d rgb_distortion;

	cv::Mat1d proj_intrinsics;
	cv::Mat1d proj_distortion;

	// stereo transform.
	cv::Mat1d R, T;
	cv::Mat1d rgb_R, rgb_T;
};

bool compX(const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; }
bool compY(const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; }

void detectSinglePrintedChessboard(const Context& context, const std::string& imageName, int width, int height, cv::Mat& frame, std::vector<cv::Point2f>& corners){

	float minX, minY, maxX, maxY;
	float offset = 12;
	
	calibrationCorners(imageName, "corners", width, height, corners, frame, 1);

	if (corners.size() == 0)
		return;
	
	minX = std::min_element(corners.begin(), corners.end(), compX)->x;
	maxX = std::max_element(corners.begin(), corners.end(), compX)->x;
	minY = std::min_element(corners.begin(), corners.end(), compY)->y;
	maxY = std::max_element(corners.begin(), corners.end(), compY)->y;
	
	minX = std::max(minX-offset, 0.f);
	maxX = std::min(maxX+offset, (float)context.image_size.width);
	minY = std::max(minY-offset, 0.f);
	maxY = std::min(maxY+offset, (float)context.image_size.height);
	//Make a rectangle
	cv::Rect roi(minX, minY, maxX-minX, maxY-minY);
	//Point a cv::Mat header at it (no allocation is done)
	cv::Mat image_roi = frame(roi);
	image_roi = cv::Scalar(255, 255, 255);

}

// Detect chessboard corners (with subpixel refinement).
// Note: Returns 1 if chessboard is found, 0 otherwise.
bool detectPrintedChessboards(const Context& context, const std::string& imageName, const cv::Mat& frame, std::vector<cv::Point2f>& corners){

	cv::Mat copy = frame.clone();
	
	// Find chessboard corners.
	std::vector<cv::Point2f> corners0, corners1, corners2, corners3, corners4;
	
	detectSinglePrintedChessboard(context, imageName + ".pattern", context.opt_pattern_width(), context.opt_pattern_height(), copy, corners0);
	detectSinglePrintedChessboard(context, imageName + ".3x6", context.rgb_pattern_size_3x6.width, context.rgb_pattern_size_3x6.height, copy, corners1);
	detectSinglePrintedChessboard(context, imageName + ".3x5", context.rgb_pattern_size_3x5.width, context.rgb_pattern_size_3x5.height, copy, corners2);
	detectSinglePrintedChessboard(context, imageName + ".3x4", context.rgb_pattern_size_3x4.width, context.rgb_pattern_size_3x4.height, copy, corners3);
	detectSinglePrintedChessboard(context, imageName + ".3x3", context.rgb_pattern_size_3x3.width, context.rgb_pattern_size_3x3.height, copy, corners4);


	
	bool ret = corners1.size() != 0 && corners2.size() != 0 && corners3.size() != 0 && corners4.size() != 0;

	if (!ret) {
		return ret;
	}
	
	corners.insert(corners.end(), corners1.begin(), corners1.end());
	corners.insert(corners.end(), corners2.begin(), corners2.end());
	corners.insert(corners.end(), corners3.begin(), corners3.end());
	corners.insert(corners.end(), corners4.begin(), corners4.end());
	
	// Return without errors (i.e., chessboard was found).
	return ret;
}

void projectorCalibrationPattern(const Context& context, std::vector<cv::Point2f>& pattern_points) {

	
	std::vector< std::vector<cv::Point3f> > pattern_points_3x3, pattern_points_3x4, pattern_points_3x5, pattern_points_3x6;
	
	calibrationPattern(pattern_points_3x3, context.rgb_pattern_size_3x3.width, context.rgb_pattern_size_3x3.height, context.opt_square_size(), 1);
	calibrationPattern(pattern_points_3x4, context.rgb_pattern_size_3x4.width, context.rgb_pattern_size_3x4.height, context.opt_square_size(), 1);
	calibrationPattern(pattern_points_3x5, context.rgb_pattern_size_3x5.width, context.rgb_pattern_size_3x5.height, context.opt_square_size(), 1);
	calibrationPattern(pattern_points_3x6, context.rgb_pattern_size_3x6.width, context.rgb_pattern_size_3x6.height, context.opt_square_size(), 1);

	pattern_points.resize(pattern_points_3x3[0].size() + pattern_points_3x4[0].size() + pattern_points_3x5[0].size() + pattern_points_3x6[0].size());
	int i = 0;

	for (std::vector<cv::Point3f>::iterator it = pattern_points_3x6[0].begin(); it != pattern_points_3x6[0].end(); it++) {
		pattern_points[i].x = it->x + context.opt_offsetX_3x6();
		pattern_points[i].y = it->y + context.opt_offsetY_3x6();
		i++;
	}

	for (std::vector<cv::Point3f>::iterator it = pattern_points_3x5[0].begin(); it != pattern_points_3x5[0].end(); it++) {
		pattern_points[i].x = it->x + context.opt_offsetX_3x5();
		pattern_points[i].y = it->y + context.opt_offsetY_3x5();
		i++;
	}

	for (std::vector<cv::Point3f>::iterator it = pattern_points_3x4[0].begin(); it != pattern_points_3x4[0].end(); it++) {
		pattern_points[i].x = it->x + context.opt_offsetX_3x4();
		pattern_points[i].y = it->y + context.opt_offsetY_3x4();
		i++;
	}
	
	for (std::vector<cv::Point3f>::iterator it = pattern_points_3x3[0].begin(); it != pattern_points_3x3[0].end(); it++) {
		pattern_points[i].x = it->x + context.opt_offsetX_3x3();
		pattern_points[i].y = it->y + context.opt_offsetY_3x3();
		i++;
	}
}

void computeHomographies_rgb(const Context& context, std::vector<cv::Mat1d*>& homographies)
{
	std::vector<cv::Point2f> pattern_points;
	projectorCalibrationPattern(context, pattern_points);
	
	std::vector< std::vector<cv::Point2f> > good_corners;
	homographies.resize(context.images_list.size());

	for (int i_image = 0; i_image < context.images_list.size(); ++i_image)
	{
		QString filename = context.images_list[i_image];
		QDir cur_image_dir (context.images_dir.absoluteFilePath(filename));

        std::string full_filename;
        if (cur_image_dir.exists("raw/color.png"))
            full_filename = cur_image_dir.absoluteFilePath("raw/color.png").toStdString();
        else if (cur_image_dir.exists("raw/color.bmp"))
            full_filename = cur_image_dir.absoluteFilePath("raw/color.bmp").toStdString();
        else
        {
            ntk::fatal_error(("Cannot load " + full_filename).c_str());
        }
		ntk_dbg_print(full_filename, 1);

		cv::Mat3b image = cv::imread(full_filename);
		ntk_ensure(image.data, "Could not load color image");

		cv::Mat3b undistorted_image;
		undistort(image, undistorted_image, context.rgb_intrinsics, context.rgb_distortion);

		std::vector<cv::Point2f> current_view_corners;
		
		bool found = detectPrintedChessboards(context, full_filename, undistorted_image, current_view_corners);
		
		if (found) {
			cv::Mat current_mat(current_view_corners);
			homographies[i_image] = new cv::Mat1d();
 			*(homographies[i_image]) = cv::findHomography(current_mat, cv::Mat(pattern_points), CV_RANSAC, 0.01);
            // 			if (i_image == 0) {
            // 				cv::Mat current_pattern;
            // 				cv::perspectiveTransform(current_mat, current_pattern, *(homographies[i_image]));
            // 				std::cout << current_mat.rows << "\t" << current_mat.cols << "\t" << current_mat.channels() << std::endl;
            // 				std::cout << current_pattern.rows << "\t" << current_pattern.cols << std::endl;
            // 				int k = 33;
            // 				std::cout << pattern_points[k].x << "\t" << pattern_points[k].y << std::endl;
            // 				std::cout << current_mat.at<cv::Point2f>(k, 0).x << "\t" << current_mat.at<cv::Point2f>(k, 0).y << std::endl;
            // 				std::cout << current_pattern.at<cv::Point2f>(k, 0).x << "\t" << current_pattern.at<cv::Point2f>(k, 0).y << std::endl;
            // 			}
		} else {
			homographies[i_image] = NULL;
		}
	}
}

void undistort_projector(const Context& context, std::vector< std::vector<cv::Point2f> >& stereo_corners)
{
	stereo_corners.resize(context.images_list.size());
	
	for (int i_image = 0; i_image < context.images_list.size(); ++i_image)
	{
		QString filename = context.images_list[i_image];
		QDir cur_image_dir (context.images_dir.absoluteFilePath(filename));
		std::string full_filename = cur_image_dir.absoluteFilePath("raw/color.png").toStdString();
		ntk_dbg_print(full_filename, 1);
		cv::Mat3b image = cv::imread(full_filename);
		ntk_ensure(image.data, "Could not load color image");

		cv::Mat3b undistorted_image;
		undistort(image, undistorted_image, context.rgb_intrinsics, context.rgb_distortion);

		std::vector<cv::Point2f> current_view_corners;
		calibrationCorners(full_filename, "corners",
						   context.opt_pattern_width(), context.opt_pattern_height(),
						   current_view_corners, undistorted_image, 1);

		if (current_view_corners.size() == (context.opt_pattern_width()*context.opt_pattern_height())) {
			stereo_corners[i_image] = current_view_corners;
		} else {
			stereo_corners[i_image].resize(0);
		}
	}
}

void find_projector_image(const Context& context, int nImages, std::vector< std::vector<cv::Point2f> >& corners) {
	
	float offsetX = (context.proj_size.width - (context.opt_pattern_width()-1) * context.opt_pattern_pixels()) / 2;
	float offsetY = (context.proj_size.height - (context.opt_pattern_height()-1) * context.opt_pattern_pixels()) / 2;
	
	std::vector< std::vector<cv::Point3f> > corners3d;
	calibrationPattern(corners3d, context.opt_pattern_width(), context.opt_pattern_height(), context.opt_pattern_pixels(), nImages);
	
	corners.resize(corners3d.size());

	for (int i = 0; i < corners3d.size(); i++) {
		
		corners[i].resize(corners3d[i].size());
		
		for (int j = 0; j < corners3d[i].size(); j++) {
			// 			corners[i][j].x = corners3d[i][j].x + offsetX;
			// 			corners[i][j].y = corners3d[i][j].y + offsetY;
			corners[i][j].x = corners3d[i][corners3d[i].size()-1-j].x + offsetX;			
			corners[i][j].y = corners3d[i][corners3d[i].size()-1-j].y + offsetY;			
			// 			corners[i][j].x = context.proj_size.width - corners3d[i][j].x - offsetX;
			//  			corners[i][j].y = context.proj_size.height - corners3d[i][j].y - offsetY;
		}
	}
}

void calibrate_projector(Context& context, const std::vector<cv::Mat1d*>& homographies,
                         const std::vector< std::vector<cv::Point2f> >& undistorted_proj_corners)
{
    ntk_assert(undistorted_proj_corners.size() == homographies.size(), "Size should be equal.");
    std::vector< std::vector<cv::Point2f> > undistorted_good_proj;
	std::vector< std::vector<cv::Point2f> > undistorted_good_rgb;
	std::vector< std::vector<cv::Point2f> > proj_pattern;

	foreach_idx(i, undistorted_proj_corners)
	{
		if (homographies[i] && undistorted_proj_corners[i].size() > 0)
		{
			ntk_assert(homographies[i] && undistorted_proj_corners[i].size() > 0, "Homographies and projected corners do not match.");

			undistorted_good_rgb.push_back(undistorted_proj_corners[i]);

			cv::Mat current_pattern;
			cv::perspectiveTransform(cv::Mat(undistorted_proj_corners[i]), current_pattern, *(homographies[i]));
			proj_pattern.push_back(current_pattern);
		}
	}

	std::vector< std::vector<cv::Point3f> > proj_pattern_3d(proj_pattern.size());
	foreach_idx(i, proj_pattern) {
		
		proj_pattern_3d[i].resize(proj_pattern[i].size());
		
		foreach_idx(k, proj_pattern[i]) {
			proj_pattern_3d[i][k].x = proj_pattern[i][k].x;
			proj_pattern_3d[i][k].y = -proj_pattern[i][k].y;
			proj_pattern_3d[i][k].z = 0.;
			
			// 			if (i == 0) {
			// 				std::cout << proj_pattern_3d[i][k] << std::endl;
			// 			}
		}
	}

	find_projector_image(context, proj_pattern.size(), undistorted_good_proj);
	
	//  	int flags = CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3;
	int flags = CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3;
	// 	int flags = 0;

	std::vector<cv::Mat> rvecs, tvecs;
	double errorInt = calibrateCamera(proj_pattern_3d, undistorted_good_proj, context.proj_size,
									  context.proj_intrinsics, context.proj_distortion,
									  rvecs, tvecs, flags);
	
	std::cout << context.proj_intrinsics << std::endl;
	std::cout << context.proj_distortion << std::endl;
	
	ntk_dbg_print(errorInt, 1);

	cv::Mat E(3,3,CV_64F),F(3,3,CV_64F);
	cv::Mat zero_dist (context.proj_distortion.size(), context.proj_distortion.type());
	zero_dist = cv::Scalar(0);

	
	stereoCalibrate(proj_pattern_3d,
					undistorted_good_proj, undistorted_good_rgb,
					context.proj_intrinsics, context.proj_distortion,
					context.rgb_intrinsics, zero_dist,
					context.proj_size,
					context.R, context.T, E, F,
					cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 50, 1e-6),
					cv::CALIB_FIX_INTRINSIC);

	double error = computeCalibrationError(F, undistorted_good_proj, undistorted_good_rgb);
	std::cout << "Average pixel reprojection error: " << error << std::endl;
	
	// 	std::cout << context.R << std::endl;
	// 	std::cout << context.T << std::endl;
	
	// 	cv::Mat1d H(4, 4);
	// 	setIdentity(H);
	//   cv::Mat1d H_rot = H(cv::Rect(0,0,3,3));
	//   context.R.copyTo(H_rot);
	//   H(0,3) = context.T(0,0);
	//   H(1,3) = context.T(1,0);
	//   H(2,3) = context.T(2,0);
	//
	// 	cv::Mat1d to_open_cv (4,4);
	//   setIdentity(to_open_cv);
	//   to_open_cv(1,1) = -1;
	//   to_open_cv(2,2) = -1;
	//   cv::Mat1d from_open_cv = to_open_cv.inv();
	//
	// 	std::cout << "H" << std::endl;
	// 	std::cout << H << std::endl;
	// 	H = from_open_cv * H;
	// 	std::cout << H << std::endl;
	//
	// 	H_rot.copyTo(context.R);
	//   context.T(0,0) = H(0,3);
	//   context.T(1,0) = H(1,3);
	//   context.T(2,0) = H(2,3);

	context.T =  context.rgb_R * context.T - context.rgb_T;
	context.R =  context.rgb_R * context.R;

	// 	std::cout << "---" << std::endl;
	// 	std::cout << context.R << std::endl;
	// 	std::cout << context.T << std::endl;
}

void writeNestkMatrix(const Context& context)
{
    cv::FileStorage output_file (context.opt_output_file(), CV_STORAGE_WRITE);
    writeMatrix(output_file, "proj_intrinsics", context.proj_intrinsics);
    writeMatrix(output_file, "proj_distortion", context.proj_distortion);
    writeMatrix(output_file, "R", context.R);
    writeMatrix(output_file, "T", context.T);
    cv::Mat1i size_matrix(1,2);
    size_matrix(0,0) = context.proj_size.width;
    size_matrix(0,1) = context.proj_size.height;
    writeMatrix(output_file, "proj_size", size_matrix);

    output_file.release();
}

void writeROSMatrix(const Context& context)
{
    // Reload nestk calibration file.
    ProjectorCalibration calib;
    calib.loadFromFile(context.opt_output_file());
	
	cv::Vec3f euler_angles = calib.pose->cvEulerRotation();
	// 	std::cout << euler_angles[0] << std::endl;
	// 	std::cout << euler_angles[1] << std::endl;
	// 	std::cout << euler_angles[2] << std::endl;

	cv::Mat1f proj = calib.pose->cvProjectionMatrix();
	cv::Mat1f identity(3,3); setIdentity(identity);
	
	cv::FileStorage ros_depth_file ("calibration_proj.yaml", CV_STORAGE_WRITE);
	writeMatrix(ros_depth_file, "camera_matrix", context.proj_intrinsics);
	writeMatrix(ros_depth_file, "distortion_coefficients", context.proj_distortion);
	writeMatrix(ros_depth_file, "rectification_matrix", identity);
	writeMatrix(ros_depth_file, "projection_matrix", proj);
	ros_depth_file.release();
}

void writeMatlabMatrix(const Context& context)
{
	// 	std::ofstream file("calibration_proj.matlab");
	std::ofstream file("matlab/calibration_proj.matlab");

	RGBDCalibration calib;
	calib.loadFromFile(context.opt_input_file());
	
	file << calib.depth_intrinsics(0, 0) << "\t" << calib.depth_intrinsics(0, 1) << "\t" << calib.depth_intrinsics(0, 2) << "\t" << 0 << std::endl;
	file << calib.depth_intrinsics(1, 0) << "\t" << calib.depth_intrinsics(1, 1) << "\t" << calib.depth_intrinsics(1, 2) << "\t" << 0 << std::endl;
	file << calib.depth_intrinsics(2, 0) << "\t" << calib.depth_intrinsics(2, 1) << "\t" << calib.depth_intrinsics(2, 2) << "\t" << 0 << std::endl;
	
	file << 1 << "\t" << 0 << "\t" << 0 << "\t" << 0 << std::endl;
	file << 0 << "\t" << 1 << "\t" << 0 << "\t" << 0 << std::endl;
	file << 0 << "\t" << 0 << "\t" << 1 << "\t" << 0 << std::endl;
	
	file << context.rgb_intrinsics(0, 0) << "\t" << context.rgb_intrinsics(0, 1) << "\t" << context.rgb_intrinsics(0, 2) << "\t" << 0 << std::endl;
	file << context.rgb_intrinsics(1, 0) << "\t" << context.rgb_intrinsics(1, 1) << "\t" << context.rgb_intrinsics(1, 2) << "\t" << 0 << std::endl;
	file << context.rgb_intrinsics(2, 0) << "\t" << context.rgb_intrinsics(2, 1) << "\t" << context.rgb_intrinsics(2, 2) << "\t" << 0 << std::endl;

	file << context.rgb_R(0, 0) << "\t" << context.rgb_R(0, 1) << "\t" << context.rgb_R(0, 2) << "\t" << context.rgb_T(0, 0) << std::endl;
	file << context.rgb_R(1, 0) << "\t" << context.rgb_R(1, 1) << "\t" << context.rgb_R(1, 2) << "\t" << context.rgb_T(1, 0) << std::endl;
	file << context.rgb_R(2, 0) << "\t" << context.rgb_R(2, 1) << "\t" << context.rgb_R(2, 2) << "\t" << context.rgb_T(2, 0) << std::endl;

	file << context.proj_intrinsics(0, 0) << "\t" << context.proj_intrinsics(0, 1) << "\t" << context.proj_intrinsics(0, 2) << "\t" << 0 << std::endl;
	file << context.proj_intrinsics(1, 0) << "\t" << context.proj_intrinsics(1, 1) << "\t" << context.proj_intrinsics(1, 2) << "\t" << 0 << std::endl;
	file << context.proj_intrinsics(2, 0) << "\t" << context.proj_intrinsics(2, 1) << "\t" << context.proj_intrinsics(2, 2) << "\t" << 0 << std::endl;
	
	file << context.R(0, 0) << "\t" << context.R(0, 1) << "\t" << context.R(0, 2) << "\t" << context.T(0, 0) << std::endl;
	file << context.R(1, 0) << "\t" << context.R(1, 1) << "\t" << context.R(1, 2) << "\t" << context.T(1, 0) << std::endl;
	file << context.R(2, 0) << "\t" << context.R(2, 1) << "\t" << context.R(2, 2) << "\t" << context.T(2, 0) << std::endl;

	
}

int main(int argc, char** argv)
{
	Context context;
	
	arg_base::set_help_option("--help");
	arg_parse(argc, argv);
	ntk::ntk_debug_level = 1;

        context.proj_size.width = context.opt_projector_width();
	context.proj_size.height = context.opt_projector_height();

        context.proj_pattern_size.width = context.opt_pattern_width();
	context.proj_pattern_size.height = context.opt_pattern_height();

	cv::namedWindow("corners");

	context.images_dir = QDir(context.opt_image_directory());
	ntk_ensure(context.images_dir.exists(), (context.images_dir.absolutePath() + " is not a directory.").toAscii());
    context.images_list = context.images_dir.entryList(QStringList("view????*"), QDir::Dirs, QDir::Name);
	
	RGBDCalibration calib;
	calib.loadFromFile(context.opt_input_file());
	context.rgb_intrinsics = calib.rgb_intrinsics;
	context.rgb_distortion = calib.rgb_distortion;
	context.rgb_R = calib.R;
	context.rgb_T = calib.T;

	std::vector<cv::Mat1d*> homographies;
	std::vector< std::vector<cv::Point2f> > undistorted_proj_corners;

	computeHomographies_rgb(context, homographies);
	undistort_projector(context, undistorted_proj_corners);
	calibrate_projector(context, homographies, undistorted_proj_corners);

	writeNestkMatrix(context);
	writeROSMatrix(context);
	writeMatlabMatrix(context);
	
	for (std::vector<cv::Mat1d*>::iterator it = homographies.begin(); it != homographies.end(); it++) {
		delete (*it);
	}

	return 0;
}

