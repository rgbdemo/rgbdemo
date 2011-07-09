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

#include "PeopleTracker.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/mesh/mesh_renderer.h>
#include <ntk/utils/time.h>
#include <QApplication>

using namespace ntk;
using namespace cv;


void PeopleTrackerParameters::loadFromYamlFile(const std::string &filename)
{
    FileStorage params_file (filename, CV_STORAGE_READ);
    ntk_throw_exception_if(!params_file.isOpened(), "Could not open " + filename);

    for (int i = 0; i < 4; ++i)
    {
        Vec3f v;
        read_from_yaml(params_file[cv::format("working_zone_%i", i)], v);
        initial_working_zone[i] = v;
    }

    read_from_yaml(params_file["min_correlation_threshold"], min_correlation_threshold);
    read_from_yaml(params_file["max_bbox_overlap"], max_bbox_overlap);
    read_from_yaml(params_file["min_inter_person_dist"], min_inter_person_dist);


    cv::Mat1f raw_template_img;
    readMatrix(params_file, "person_template", raw_template_img);
    // read_from_yaml(params_file["person_template"], raw_template_img);

    cv::Rect template_rect;
    read_from_yaml(params_file["person_template_roi"], template_rect);

    template_img.create(template_rect.height, template_rect.width);
    raw_template_img(template_rect).copyTo(template_img);
    read_from_yaml(params_file["person_template_pixel_size"], template_pixel_size);

    min_template_depth = FLT_MAX;
    for_all_rc(template_img)
    {
        if (template_img(r,c) > 1e-5)
            min_template_depth = std::min(min_template_depth,
                                          template_img(r,c));
    }
    // imshow_normalized("template", template_img);
    // waitKey(10);
}

PeopleTracker::PeopleTracker(const PeopleTrackerParameters& parameters)
    : m_parameters(parameters)
{
    m_background_depth.create(640,480);
    m_background_depth = FLT_MAX;

    std::copy(m_parameters.initial_working_zone, m_parameters.initial_working_zone+4,
              m_working_zone);
}

void PeopleTracker::saveBackgroundInfoToFile(const std::string& filename) const
{
    FileStorage tracker_file (filename, CV_STORAGE_WRITE);

    write_to_yaml(tracker_file, "ground_plane", m_ground_plane);

    cvStartWriteStruct(*tracker_file, "virtual_top_view", CV_NODE_MAP);
    m_virtual_top_view.saveToYaml(tracker_file);
    cvEndWriteStruct(*tracker_file);

    write_to_yaml(tracker_file, "top_image_width", m_top_image_width);
    write_to_yaml(tracker_file, "top_image_height", m_top_image_height);
    write_to_yaml(tracker_file, "top_image_pixel_size", m_top_pixel_size);

    for (int i = 0; i < 4; ++i)
    {
        write_to_yaml(tracker_file, cv::format("working_zone_%i", i),
                      (Vec3f)m_working_zone[i]);
    }

    write_to_yaml(tracker_file, "background_depth", m_background_depth);
}

void PeopleTracker::loadBackgroundInfoFromFile(const std::string& filename)
{
    FileStorage tracker_file (filename, CV_STORAGE_READ);

    read_from_yaml(tracker_file["ground_plane"] , m_ground_plane);

    for (int i = 0; i < 4; ++i)
    {
        Vec3f v;
        read_from_yaml(tracker_file[cv::format("working_zone_%i", i)], v);
        m_working_zone[i] = v;
    }

    read_from_yaml(tracker_file["background_depth"], m_background_depth);

    m_virtual_top_view.loadFromYaml(tracker_file["virtual_top_view"]);
    read_from_yaml(tracker_file["top_image_width"], m_top_image_width);
    read_from_yaml(tracker_file["top_image_height"], m_top_image_height);
    read_from_yaml(tracker_file["top_image_pixel_size"], m_top_pixel_size);
}

void PeopleTracker::estimateBackground(const ntk::RGBDImage& image)
{
    image.depth().copyTo(m_background_depth);
    image.copyTo(m_foreground_image);

    estimateGroundPlaneFromUserInput(image);
    estimateInitialWorkingZone(image);
    computeVirtualTopViewPose();
    computeVirtualTopView();

    estimateWorkingZoneFromUserInput(image);
    computeVirtualTopViewPose();
}

namespace
{

struct TrackerGroundMouseData
{
    std::string window_name;
    cv::Mat3b image;
    std::vector<Point> points;
};

static void on_tracker_ground_mouse(int event, int x, int y, int flags, void *void_data)
{
    if (event != CV_EVENT_RBUTTONUP)
        return;

    TrackerGroundMouseData* data = (TrackerGroundMouseData*)void_data;
    data->points.push_back(Point(x, y));
    circle(data->image, Point(x,y), 5, Scalar(255,0,0,255));
    imshow(data->window_name, data->image);
}

}

void PeopleTracker::estimateInitialWorkingZone(const ntk::RGBDImage& image)
{ 
    for (int i = 0; i < 4; ++i)
    {
        // Intersect user-selected point with ground plane.
        Point3f p0 = m_parameters.initial_working_zone[i]-Point3f(m_ground_plane.normal()*1000.f);
        Point3f p1 = m_parameters.initial_working_zone[i]+Point3f(m_ground_plane.normal()*1000.f);
        m_working_zone[i] = m_ground_plane.intersectionWithLine(p0, p1);
    }
}

void PeopleTracker::estimateGroundPlaneFromUserInput(const ntk::RGBDImage& image)
{
    // Estimate ground plane equation asking 3 points to the user.
    TrackerGroundMouseData ground_mouse_data;
    ground_mouse_data.window_name = "Select 3 points to estimate the ground plane (right click)";
    namedWindow(ground_mouse_data.window_name,
                CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
    ground_mouse_data.image = toMat3b(normalize_toMat1b(image.depth()));
    imshow(ground_mouse_data.window_name, ground_mouse_data.image);
    setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
    while (ground_mouse_data.points.size() != 3)
    {
        waitKey(30);
        QApplication::processEvents();
    }
    ntk_dbg(0) << "I am here!";

    Point3f p0, p1, p2;
    p0 = image.calibration()->depth_pose->unprojectFromImage(ground_mouse_data.points[0],
                                                             image.depth()(ground_mouse_data.points[0]));
    p1 = image.calibration()->depth_pose->unprojectFromImage(ground_mouse_data.points[1],
                                                             image.depth()(ground_mouse_data.points[1]));
    p2 = image.calibration()->depth_pose->unprojectFromImage(ground_mouse_data.points[2],
                                                             image.depth()(ground_mouse_data.points[2]));
    destroyWindow(ground_mouse_data.window_name);
    Vec3f normal = -(p1-p0).cross(p2-p0);
    normalize(normal);
    if (normal[2] < 0)
        normal = -normal;
    m_ground_plane = Plane(normal, p0);
}

void PeopleTracker :: estimateWorkingZoneFromUserInput(const ntk::RGBDImage& image)
{
    TrackerGroundMouseData ground_mouse_data;
    ground_mouse_data.window_name = "Choose 4 points to define the working zone (right click).";
    namedWindow(ground_mouse_data.window_name,
                CV_WINDOW_NORMAL|CV_WINDOW_KEEPRATIO|CV_GUI_EXPANDED);
    ground_mouse_data.image = toMat3b(normalize_toMat1b(m_current_top_view));
    imshow(ground_mouse_data.window_name, ground_mouse_data.image);
    setMouseCallback(ground_mouse_data.window_name, on_tracker_ground_mouse, &ground_mouse_data);
    while (ground_mouse_data.points.size() != 4)
        waitKey(30);
    destroyWindow(ground_mouse_data.window_name);

    Pose3D depth_pose = m_virtual_top_view; depth_pose.invert();
    for (int i = 0; i < 4; ++i)
    {
        Point3f p3d (ground_mouse_data.points[i].x,
                     ground_mouse_data.points[i].y,
                     -1);
        p3d = depth_pose.unprojectFromImage(p3d);

        // Intersect user-selected point with ground plane.
        Point3f p0 = p3d-Point3f(m_ground_plane.normal()*1000.f);
        Point3f p1 = p3d+Point3f(m_ground_plane.normal()*1000.f);
        m_working_zone[i] = m_ground_plane.intersectionWithLine(p0, p1);
    }
}

void PeopleTracker::processNewImage(const ntk::RGBDImage& image)
{
    m_current_image = image;
    computeForeground();
    computeVirtualTopView();
    detectPeople();
}

static void on_mouse_show_value(int event, int x, int y, int flags, void *void_data)
{
    if (event != CV_EVENT_MOUSEMOVE)
        return;

    cv::Mat* img = (Mat*)void_data;
    switch(img->type())
    {
    case CV_32FC1:
        ntk_dbg(0) << img->at<float>(y,x);
        break;
    default:
        break;
    };
}

void PeopleTracker::computeVirtualTopView()
{
    if (!m_virtual_top_view.isValid())
        return;

    Pose3D to_virtual_view = m_virtual_top_view;
    to_virtual_view.invert();

    const cv::Mat1f& depth_im = m_foreground_image.depth();
    const cv::Mat1b& mask_im = m_foreground_image.depthMask();
    m_current_top_view.create(m_top_image_height, m_top_image_width);
    m_current_top_view = 0.f;
    TimeCount top_view_timer("Top view computation");
    for_all_rc(mask_im)
    {
        if (!mask_im(r,c))
            continue;

        const float d = depth_im(r,c);
        Point3f p3d = m_current_image.calibration()->depth_pose->unprojectFromImage(Point3f(c,r,d));
        Point3f top = to_virtual_view.projectToImage(p3d);
        if (!is_yx_in_range(m_current_top_view, top.y, top.x))
            continue;

        // z-test
        double prev_depth = m_current_top_view(top.y, top.x);
        if (prev_depth < 1e-5 || top.z < prev_depth)
            m_current_top_view(top.y, top.x) = top.z;
    }
    // imshow_normalized("top_image", m_current_top_view);
    // setMouseCallback("top_image", on_mouse_show_value, &m_current_top_view);
    // imwrite_yml("person_template.yml", m_current_top_view);
    // imwrite_normalized("person_template.png", m_current_top_view);
    // waitKey(10);
    top_view_timer.stop();
}

void PeopleTracker::computeForeground()
{
    const double max_depth_difference = 0.1;

    TimeCount top_view_timer("Compute foreground");
    m_current_image.copyTo(m_foreground_image);
    Mat1b& fg_depth_mask = m_foreground_image.depthMaskRef();
    const Mat1f& fg_depth_im = m_foreground_image.depth();
    const Mat1f& bg_depth_im = m_background_depth;
    for_all_rc(fg_depth_mask)
    {
        float bg_depth = bg_depth_im(r,c);
        float fg_depth = fg_depth_im(r,c);
        if (std::abs(bg_depth-fg_depth) < max_depth_difference || bg_depth < 1e-5)
            fg_depth_mask(r,c) = 0;
    }

    cv::morphologyEx(fg_depth_mask, fg_depth_mask,
                     cv::MORPH_OPEN,
                     getStructuringElement(cv::MORPH_RECT,
                                           cv::Size(5,5)));
    top_view_timer.stop();
}

double PeopleTracker::correlationValue(const cv::Mat1f& img,
                                       const cv::Mat1f& pattern,
                                       const cv::Point& topleft)
{
    double corr = 0;
    int n = 0;

    double min_depth = FLT_MAX;
    for (int r = topleft.y; r < topleft.y + pattern.rows && r < img.rows; ++r)
        for (int c = topleft.x; c < topleft.x + pattern.cols && c < img.cols; ++c)
        {
            double d = img(r,c);
            if (d < 1e-5)
                continue;
            min_depth = std::min(min_depth, d);
        }

    for (int r = topleft.y; r < topleft.y + pattern.rows && r < img.rows; ++r)
        for (int c = topleft.x; c < topleft.x + pattern.cols && c < img.cols; ++c)
        {
            if (!is_yx_in_range(img, r, c))
                continue;
            corr += img(r,c) * pattern(r-topleft.y,c-topleft.x);
            ++n;
        }
    if (n > 0)
        corr /= n;
    return corr;
}

struct PeopleTracker :: DetectionFilterPredicate
{
    DetectionFilterPredicate(const PeopleTrackerParameters& params,
                             float pixel_size)
        : m_params(params), m_pixel_size(pixel_size)
    {}

    bool operator()(const PersonDetection& lhs,
                    const PersonDetection& rhs) const
    {
        double overlap_ratio = ntk::overlap_ratio(lhs.bbox, rhs.bbox);
        float dist = sqrt(ntk::math::sqr(lhs.highest_point_in_image.x - rhs.highest_point_in_image.x)
                          + ntk::math::sqr(lhs.highest_point_in_image.y - rhs.highest_point_in_image.y));
        if (overlap_ratio > m_params.max_bbox_overlap
                || dist < m_params.min_inter_person_dist/m_pixel_size)
        {
            ntk_dbg_print(overlap_ratio, 2);
            return true;
        }
        else
            return false;
    }

private:
    const PeopleTrackerParameters& m_params;
    float m_pixel_size;
};

void PeopleTracker::detectPeople()
{
    Pose3D to_virtual_top_view = m_virtual_top_view;
    to_virtual_top_view.invert();

    const double corr_threshold = m_parameters.min_correlation_threshold;

    if (!m_current_top_view.data)
        return;

    cv::Mat1f current_template;
    const float pixel_size_factor = m_parameters.template_pixel_size/m_top_pixel_size;
    resize( m_parameters.template_img, current_template, Size(0,0), pixel_size_factor, pixel_size_factor);
    int tpl_width = current_template.cols;
    int tpl_height = current_template.rows;

    // imshow_normalized("normalized template", current_template);

    cv::Mat1f cropped_corr_image;
    ntk::TimeCount detect_tc("Template matching");
    if (m_current_top_view.cols < current_template.cols || m_current_top_view.rows < current_template.rows)
    {
        ntk_dbg(0) << "Warning: view size if smaller than template size, cannot find people.";
        ntk_dbg(0) << "Increase the working zone size.";
        return;
    }
    matchTemplate(m_current_top_view, current_template, cropped_corr_image, CV_TM_CCORR);
    // Restore the original image size using black borders.
    cv::Mat1f corr_image(m_current_top_view.size());
    corr_image = 0.f;
    cv::Mat1f corr_image_roi = corr_image(Rect(current_template.cols/2,
                                               current_template.rows/2,
                                               cropped_corr_image.cols,
                                               cropped_corr_image.rows));
    cropped_corr_image.copyTo(corr_image_roi);
    detect_tc.stop();

    // imshow_normalized("correlation", corr_image);

    ntk::TimeCount find_max_timer("Find maxima");
    cv::Mat1b element(7,7);
    element << 1,1,1,1,1,1,1,
            1,1,1,0,1,1,1,
            1,1,1,1,1,1,1;
    cv::Mat1f dilated_image;
    cv::morphologyEx(corr_image, dilated_image,
                     cv::MORPH_DILATE,
                     element);
    // imwrite_normalized("dilated.png", dilated_image);

    cv::Mat1b maxima(dilated_image.size());
    maxima = 0u;

    std::set<PersonDetection> detections;

    for_all_rc(dilated_image)
    {
        if (corr_image(r,c) > corr_threshold && corr_image(r,c) > (dilated_image(r,c)*0.99))
        {
            ntk_dbg_print(dilated_image(r,c), 2);
            ntk_dbg_print(corr_image(r,c), 2);
            PersonDetection detection;
            detection.enabled = true;
            detection.score = corr_image(r,c);
            detection.bbox = Rect(Point(c,r) - Point(tpl_width/2, tpl_height/2),
                                  Size(tpl_width, tpl_height));
            float min_depth = FLT_MAX;
            Point3f min_point (0,0,0);
            for (int br = detection.bbox.y+detection.bbox.height*0.2; br < (detection.bbox.y + detection.bbox.height*0.8); ++br)
                for (int bc = detection.bbox.x+detection.bbox.width*0.2; bc < (detection.bbox.x + detection.bbox.width*0.8); ++bc)
                {
                    if (!is_yx_in_range(m_current_top_view, br, bc))
                        continue;

                    float d = m_current_top_view(br,bc);
                    if (d < 1e-5)
                        continue;
                    if (d < min_depth)
                    {
                        min_depth = d;
                        min_point = Point3f(bc, br, d);
                    }
                }
            detection.highest_point_in_image = min_point;
            detection.highest_point = to_virtual_top_view.unprojectFromImage(min_point);
            detection.person_height = 3.0 - detection.highest_point_in_image.z;
            ntk_dbg_print(detection.highest_point, 2);
            ntk_dbg_print(min_depth, 2);

            // FIXME: make this a paramters.
            // The camera is at 3meters from the ground.
            // So to detect object starting from 1m40, the threshold must be 1.60.
            if (min_depth > 1.60)
                continue;

            detections.insert(detection);
            maxima(r,c) = 255;
        }
    }
    find_max_timer.stop();
    // imwrite("debug_maxima.png", maxima);

    DetectionFilterPredicate overlap_filter (m_parameters, m_top_pixel_size);
    m_current_detections.clear();
    for (std::set<PersonDetection>::reverse_iterator it = detections.rbegin();
         it != detections.rend();
         ++it)
    {
        const PersonDetection& detection = *it;
        if (!detection.enabled)
            continue;

        std::set<PersonDetection>::reverse_iterator rhs_it = it;
        for (++rhs_it; rhs_it != detections.rend(); ++rhs_it)
        {
            if (!rhs_it->enabled)
                continue;
            if (overlap_filter(detection, *rhs_it))
            {
                PersonDetection& rhs_detection = const_cast<PersonDetection&>(*rhs_it);
                rhs_detection.enabled = false;
            }
        }
    }

    for (std::set<PersonDetection>::reverse_iterator it = detections.rbegin();
         it != detections.rend();
         ++it)
    {
        if (it->enabled)
            m_current_detections.push_back(*it);
    }

#if 1
    cv::Mat3b detections_im(corr_image.size());
    detections_im = Vec3b(0,0,0);
    foreach_const_it(it, detections, std::set<PersonDetection>)
    {
        Scalar color = it->enabled ? Scalar(0,255,0,255) : Scalar(255,0,0,255);
        rectangle(detections_im, it->bbox, color);
        circle(detections_im,
               Point(it->highest_point_in_image.x, it->highest_point_in_image.y),
               5, color);
    }
    // imshow("detections", detections_im);

    double min_corr, max_corr;
    minMaxLoc(corr_image, &min_corr, &max_corr);
    ntk_dbg_print(min_corr, 2);
    ntk_dbg_print(max_corr, 2);
#endif
    // waitKey(10);
}

void PeopleTracker::computeVirtualTopViewPose()
{
    const int view_resolution = 640;

    // FIXME: set the image size in function of the working zone size.
    // FIXME: define a X-Y resolution.
    m_virtual_top_view = *m_current_image.calibration()->depth_pose;

    cv::Vec3f camera_center (0,0,0);
    for (int i = 0; i < 4; ++i)
        camera_center += Vec3f(m_working_zone[i]);
    camera_center *= 1.f / 4;

    cv::Vec3f axis = ntk::compute_axis_angle_rotation(Vec3f(0,0,-1), -m_ground_plane.normal());
    m_virtual_top_view.applyTransformAfterRodrigues(camera_center + m_ground_plane.normal()*3.0f,
                                                    axis);

    Point3f new_p0 = m_virtual_top_view.invCameraTransform(m_working_zone[0]);
    Point3f new_p1 = m_virtual_top_view.invCameraTransform(m_working_zone[1]);
    Point3f new_orig = m_virtual_top_view.invCameraTransform(Vec3f(0,0,0));

    // Try to make an aligned rectangle with the working zone.
    Vec3f new_x = new_p1-new_p0;
    normalize(new_x);
    float alpha = acos(new_x.dot(Vec3f(1,0,0)));
    m_virtual_top_view.applyTransformBeforeRodrigues(Vec3f(0,0,0),
                                                     Vec3f(0,0,alpha));

    Point3f transformed_p[4];
    float min_x=FLT_MAX, max_x=-FLT_MAX, min_y=FLT_MAX, max_y=-FLT_MAX;
    for (int i = 0; i < 4; ++i)
    {
        transformed_p[i] = m_virtual_top_view.invCameraTransform(m_working_zone[i]);
        min_x = std::min(min_x, transformed_p[i].x);
        max_x = std::max(max_x, transformed_p[i].x);
        min_y = std::min(min_y, transformed_p[i].y);
        max_y = std::max(max_y, transformed_p[i].y);
    }

    double zone_width = max_x - min_x;
    double zone_height = max_y - min_y;

    double fy = view_resolution/zone_height; // 1024 pixels for queue length.
    double fx = fy;
    double cy = fy * zone_height / 2.0;
    double cx = fy * zone_width / 2.0;

    m_top_image_width = std::ceil(cx*2);
    m_top_image_height = std::ceil(cy*2);
    m_top_pixel_size = 1.0 / fy;

    m_virtual_top_view.setCameraParameters(fx, fy, cx, cy, true);
    ntk_dbg_print(m_virtual_top_view, 2);
}

