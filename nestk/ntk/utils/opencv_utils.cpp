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

#include "opencv_utils.h"
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <QImage>
#include <QTemporaryFile>
#include <QDir>

using namespace cv;

const NtkDebug& operator<<(const NtkDebug& stream, const cv::Mat1d& m)
{
  stream << "\n";
  for (int r = 0; r < m.rows; ++r)
  {
    for (int c = 0; c < m.cols; ++c)
      stream << m(r,c) << " ";
    stream << "\n";
  }
  return stream;
}

const NtkDebug& operator<<(const NtkDebug& stream, const cv::Mat1f& m)
{
  stream << "\n";
  for (int r = 0; r < m.rows; ++r)
  {
    for (int c = 0; c < m.cols; ++c)
      stream << m(r,c) << " ";
    stream << "\n";
  }
  return stream;
}

namespace ntk
{

  cv::Mat4b toMat4b(const cv::Mat3b& im)
  {
    Mat4b out (im.size());
    for_all_rc(im)
    {
      Vec3b v = im(r,c);
      out(r,c) = Vec4b(v[0], v[1], v[2], 255);
    }
    return out;
  }

  cv::Mat3b toMat3b(const cv::Mat4b& im)
  {
    Mat3b out (im.size());
    for_all_rc(im)
    {
      Vec4b v = im(r,c);
      out(r,c) = Vec3b(v[0], v[1], v[2]);
    }
    return out;
  }

  cv::Mat1b qimage_to_opencv(const QImage& im)
  {
    QTemporaryFile f(QDir::tempPath() + "/ntk_XXXXXX.pgm");
    bool ok = f.open(); if (!ok) qFatal("Cannot create temporary file.");
    im.save(f.fileName(), "PGM");
    return cv::imread(f.fileName().toStdString(), 0);
  }

  void opencv_to_qimage(QImage& qim, const cv::Mat1b& im)
  {
    qim = QImage(im.cols, im.rows, QImage::Format_RGB32);
    for_all_rc(im)
    {
      int v = im(r,c);
      qim.setPixel(c,r,qRgb(v,v,v));
    }
  }

  void opencv_to_qimage(QImage& qim, const cv::Mat3b& im)
  {
    qim = QImage(im.cols, im.rows, QImage::Format_RGB32);
    for_all_rc(im)
    {
      cv::Vec3b color = im(r,c);
      qim.setPixel(c,r,qRgb(color[2],color[1],color[0]));
    }
  }

  void opencv_to_qimage(QImage& qim, const cv::Mat4b& im)
  {
    qim = QImage(im.cols, im.rows, QImage::Format_ARGB32);
    for_all_rc(im)
    {
      cv::Vec4b color = im(r,c);
      qim.setPixel(c,r,qRgba(color[2],color[1],color[0], color[3]));
    }
  }

  cv::Mat4b qimage_argb_to_opencv(const QImage& im)
  {
    cv::Mat4b cv_im (im.height(), im.width());
    for_all_rc(cv_im)
    {
      QRgb pixel = im.pixel(c, r);
      cv::Vec4b color (qBlue(pixel), qGreen(pixel), qRed(pixel), qAlpha(pixel));
      cv_im(r,c) = color;
    }
    return cv_im;
  }

  void apply_mask(cv::Mat1b& im, const cv::Mat1b& mask)
  {
    if (!mask.data)
      return;
    ntk_assert(im.size() == mask.size(), "Wrong mask size");
    for_all_rc(im)
      if (mask(r,c) == 0)
        im(r,c) = 0;
  }

  void apply_mask(cv::Mat3b& im, const cv::Mat1b& mask)
  {
    if (!mask.data)
      return;
    ntk_assert(im.size() == mask.size(), "Wrong mask size");
    for_all_rc(im)
      if (mask(r,c) == 0)
        im(r,c) = Vec3b(0,0,0);
  }

  void apply_mask(cv::Mat1f& im, const cv::Mat1b& mask)
  {
    if (!mask.data)
      return;
    ntk_assert(im.size() == mask.size(), "Wrong mask size");
    for_all_rc(im)
      if (mask(r,c) == 0)
        im(r,c) = 0.f;
  }

  void read_from_yaml(cv::FileNode node, bool& b)
  {
    ntk_throw_exception_if(node.empty(), "Could not read " + node.name() + " from yaml file.");
    int i = cvReadInt(*node, -1);
    ntk_assert(i >= 0 && i <= 1, "Invalid boolean value");
    b = i;
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, bool b)
  {
    cvWriteInt(*output_file, name.c_str(), b);
  }

  void read_from_yaml(cv::FileNode node, int& i)
  {
    ntk_throw_exception_if(node.empty(), "Could not read " + node.name() + " from yaml file.");
    i = cvReadInt(*node, -1);
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, int i)
  {
    cvWriteInt(*output_file, name.c_str(), i);
  }

  void read_from_yaml(cv::FileNode node, double& b)
  {
    ntk_throw_exception_if(node.empty(), "Could not read " + node.name() + " from yaml file.");
    b = cvReadReal(*node, 0);
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, double b)
  {
    cvWriteReal(*output_file, name.c_str(), b);
  }

  void write_to_yaml(FileStorage& output_file, const std::string& name, const cv::Rect& r)
  {
    cv::Mat1f m(1,4);
    m << r.x, r.y, r.width, r.height;
    CvMat c_m = m;
    output_file.writeObj(name, &c_m);
  }

  void read_from_yaml(FileNode node, cv::Rect& r)
  {
    CvMat* c_m;
    c_m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!c_m, std::string("Could not read field ") + node.name() + " from yml file.");
    cv::Mat1f m (c_m);
    ntk_assert(m.cols == 4, "Bad Rect.");
    r = cv::Rect(m(0,0), m(0,1), m(0,2), m(0,3));
  }

  void write_to_yaml(FileStorage& output_file, const std::string& name, const cv::Vec3f& v)
  {
    cv::Mat1f m(1,3);
    std::copy(&v[0], &v[0]+3, m.ptr<float>());
    CvMat c_m = m;
    output_file.writeObj(name, &c_m);
  }

  void read_from_yaml(FileNode node, cv::Vec3f& v)
  {
    CvMat* c_m;
    c_m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!c_m, std::string("Could not read field ") + node.name() + " from yml file.");
    cv::Mat1f m (c_m);
    ntk_assert(m.cols == 3, "Bad vector.");
    std::copy(m.ptr<float>(), m.ptr<float>()+3, &v[0]);
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, const cv::Mat& matrix)
  {
    CvMat m = matrix;
    output_file.writeObj(name, &m);
  }

  void read_from_yaml(cv::FileNode node, cv::Mat& matrix)
  {
    CvMat* m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!m, std::string("Could not read field ") + node.name() + " from yml file.");
    matrix = m;
  }

  void writeMatrix(FileStorage& output_file, const std::string& name, const cv::Mat& matrix)
  {
    CvMat m = matrix;
    output_file.writeObj(name, &m);
  }

  void readMatrix(FileStorage& input_file, const std::string& name, cv::Mat& matrix)
  {
    FileNode node = input_file[name];
    CvMat* m;
    m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!m, std::string("Could not read field ") + name + " from yml file.");
    matrix = m;
  }

  void imwrite_yml(const std::string& filename, const cv::Mat& image)
  {
    IplImage tmp = image;
    cvSave(filename.c_str(), &tmp);
  }

  cv::Mat imread_yml(const std::string& filename)
  {
    IplImage* tmp = (IplImage*) cvLoad(filename.c_str());
    ntk_throw_exception_if(!tmp, "Could not load " + filename);
    return cv::Mat(tmp);
  }

  cv::Mat3b toMat3b(const cv::Mat1b& image)
  {
    cv::Mat3b tmp;
    cv::cvtColor(image, tmp, CV_GRAY2BGR);
    return tmp;
  }

  cv::Mat1b normalize_toMat1b(const cv::Mat1f& image)
  {
    cv::Mat1b tmp;
    normalize(image, tmp, 0, 255, NORM_MINMAX, 0);
    return tmp;
  }

  void imwrite_normalized(const std::string& filename, const cv::Mat1f& image)
  {
    cv::Mat1b tmp;
    normalize(image, tmp, 0, 255, NORM_MINMAX, 0);
    imwrite(filename, tmp);
  }

  void imshow_normalized(const std::string& window_name, const cv::Mat1f& image)
  {
    cv::Mat1b tmp;
    normalize(image, tmp, 0, 255, NORM_MINMAX, 0);
    imshow(window_name, tmp);
  }

  double overlap_ratio(const cv::Rect_<float>& r1, const cv::Rect_<float>& r2)
  {
    cv::Rect_<float> union12  = r1 | r2;

    cv::Rect_<float> intersection12 = r1 & r2;
    return intersection12.area()/union12.area();
  }

  void adjustRectToImage(cv::Rect& rect, const cv::Size& image_size)
  {
    rect.x = std::max(rect.x, 0);
    rect.y = std::max(rect.y, 0);
    rect.x = std::min(rect.x, image_size.width-1);
    rect.y = std::min(rect.y, image_size.height-1);
    rect.width = std::min(image_size.width-rect.x-1, rect.width);
    rect.height = std::min(image_size.height-rect.y-1, rect.height);
  }


  ntk::Rect3f bounding_box(const std::vector<cv::Point3f>& points)
  {
    ntk::Rect3f box;
    foreach_idx(i, points)
    {
      box.extendToInclude(points[i]);
    }
    return box;
  }

}
