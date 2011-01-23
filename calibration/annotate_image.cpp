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

#include <ntk/utils/opencv_utils.h>
#include <ntk/ntk.h>
#include <opencv/highgui.h>
#include <fstream>

#include <QDir>
#include <QStringList>

using namespace cv;
using namespace ntk;

namespace global
{
  ntk::arg<const char*> image_dir(0, "Calibration directory with the view???? images", 0);
  ntk::arg<const char*> image_component("--image-prefix", "Image component (color.png, intensity.png or depth.png)", "depth.png");
  cv::Mat3b im;
}

struct CalibPoints
{
  std::vector<Point2i> points;
};

void mouseCallBack(int event, int x, int y, int flags, void* param)
{
  if (event != CV_EVENT_FLAG_LBUTTON)
    return;
  CalibPoints* points = reinterpret_cast<CalibPoints*>(param);
  points->points.push_back(Point2i(x,y));
  circle(global::im, Point(x,y), 5, Scalar(255,0,0,255));
  imshow("annotator", global::im);
}

int main(int argc, char** argv)
{
  arg_base::set_help_option("--help");
  arg_parse(argc, argv);
  ntk::ntk_debug_level = 1;

  QDir image_dir (global::image_dir());
  QStringList images = image_dir.entryList(QStringList("view????"), QDir::Dirs, QDir::Name);

  namedWindow("annotator", 0);

  foreach(QString image_file, images)
  {
    std::string full_filename = image_dir.absoluteFilePath(image_file).toStdString();
    CalibPoints points;
    global::im = imread(full_filename + "/raw/" + global::image_component());
    cvSetMouseCallback("annotator", mouseCallBack, &points);
    imshow("annotator", global::im);
    cvResizeWindow("annotator", 640*2, 480*2);
    while ((cv::waitKey() & 0xff) != 27) {}
    if (points.points.size() == 4)
    {
      std::ofstream f((full_filename + "/raw/" + global::image_component() + ".calib").c_str());
      foreach_idx(i, points.points)
	{
	  f << points.points[i].x << " " << points.points[i].y << std::endl;
	}
      f.close();
    }
  }


}
