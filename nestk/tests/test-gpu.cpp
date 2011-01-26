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

#include <ntk/core.h>
#include <ntk/utils/arg.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/gpu/opencl.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/camera/calibration.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/camera/kinect_grabber.h>

#include <QApplication>

using namespace ntk;
using namespace cv;

namespace opt
{

// Command line argument.
ntk::arg<const char*> calibration_file("--calibration", "Calibration file", "kinect_calibration.yml");

}

std::string project_cl_code = opencl_stringify(
  //column major 4x4 * 4x1
  float4 matmult4(constant float* mat, float4 vec)
{
  float4 rvec;
  rvec.x = dot((float4)(mat[0],mat[4],mat[8],mat[12]),vec);
  rvec.y = dot((float4)(mat[1],mat[5],mat[9],mat[13]),vec);
  rvec.z = dot((float4)(mat[2],mat[6],mat[10],mat[14]),vec);
  rvec.w = dot((float4)(mat[3],mat[7],mat[11],mat[15]),vec);
  return rvec;
}

__kernel void project( __global float4* kin,
                      __global float4* color,
                      __global float* depth,
                      __global uchar* rgb,    //packed as 3 floats from OpenCV
                      __constant float* pt,
                      __constant float* ipt,
                      int w,
                      int h)
{
  unsigned int i = get_global_id(0);

  int c = i % w;
  int r = (int)(i / w);
  //int r = i % h;
  //int c = (int)(i / w);



  float d = depth[i];
  int irgb = i*3;
  //irgb = r*(w*3) + c*3;
  float4 col = (float4)(rgb[irgb+2]/255.f, rgb[irgb+1]/255.f, rgb[irgb]/255.f, 1.0f);
  //col.y = 1;
  color[i] = col;
  //color[i] = (float4)(d/3., d/3., d/3., 1);
  //kin[i] = (float4)(d*c/w, d*r/h, d, 1);
  kin[i] = (float4)(1.5*c/w, 1.5*r/h, 0, 1);
  //color[i] = (float4)(1,0,0,0);
  //kin[i] = (float4)(0,0,0,0);
#if 1
  //unproject from depth, in place
  float4 epix = (float4)(d*c, d*r, d, 1);
  kin[i] = matmult4(ipt, epix);
  kin[i].w = 1;


  epix = matmult4(pt, kin[i]);
  int x = (int)(epix.x/epix.z);
  int y = (int)(epix.y/epix.z);

  /*
   inline bool is_yx_in_range(const cv::Mat& image, int y, int x)
{ return (x >= 0) && (y >= 0) && (x < image.cols) && (y < image.rows); }
*/
  //if y,x in range

  if(x>=0 && y>=0 && x < w && y < h)
  {
    irgb = y*w*3 + x*3;
    color[i].x = rgb[irgb+2]/255.;
    color[i].y = rgb[irgb+1]/255.;
    color[i].z = rgb[irgb]/255.;
    //color[i] = (float4)(0,1,0,1);
  }
  else
  {
    color[i] = (float4)(0,0,1,1);
  }

#endif
}
);

int main(int argc, char ** argv)
{
  // Parse command line arguments.
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);

  ntk::ntk_debug_level = 1; // set the debug level

  CL cli;

  std::vector<float> kinect_depth(640*480);
  std::vector<uchar> kinect_rgb(640*480*3);
  std::vector<Vec4f> kinect_data(640*480);
  std::vector<Vec4f> kinect_col(640*480);
  std::vector<float> kinect_trans(16);
  Vec4f* kinect_raw_data = 0;

  // Buffer<Vec4f> cl_kinect(&cli, kinect_data, CL_MEM_WRITE_ONLY|CL_MEM_USE_HOST_PTR, BufferUseHostMemType());
  Buffer<Vec4f> cl_kinect(&cli, kinect_data, CL_MEM_WRITE_ONLY);
#if 0
  Buffer<Vec4f> cl_kinect (&cli,
                           &kinect_raw_data,
                           640*480,
                           CL_MEM_WRITE_ONLY|CL_MEM_ALLOC_HOST_PTR,
                           BufferUsePinnedMemory());
#endif
  Buffer<Vec4f> cl_kinect_col(&cli, kinect_col, CL_MEM_WRITE_ONLY);
  Buffer<float> cl_kinect_depth(&cli, kinect_depth, CL_MEM_READ_ONLY);
  Buffer<uchar> cl_kinect_rgb(&cli, kinect_rgb, CL_MEM_READ_ONLY);
  Buffer<float> cl_pt(&cli, kinect_trans, CL_MEM_READ_ONLY); //projection transforms
  Buffer<float> cl_ipt(&cli, kinect_trans, CL_MEM_READ_ONLY);

  Kernel kernel (&cli, "project", project_cl_code);
  {
    int args = 0;
    kernel.setArg(args++, cl_kinect.getDevicePtr());
    kernel.setArg(args++, cl_kinect_col.getDevicePtr());
    kernel.setArg(args++, cl_kinect_depth.getDevicePtr());
    kernel.setArg(args++, cl_kinect_rgb.getDevicePtr());
    kernel.setArg(args++, cl_pt.getDevicePtr());
    kernel.setArg(args++, cl_ipt.getDevicePtr());
    kernel.setArg(args++, 640);
    kernel.setArg(args++, 480);
  }


  RGBDCalibration calibration;
  calibration.loadFromFile(opt::calibration_file());
  Pose3D& depth_pose = *calibration.depth_pose;
  Pose3D& rgb_pose = *calibration.rgb_pose;

  RGBDProcessor processor;
  processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);
  processor.setFilterFlag(RGBDProcessor::NoAmplitudeIntensityUndistort, true);

  KinectGrabber grabber;
  grabber.initialize();
  // Tell the grabber that we have calibration data.
  grabber.setCalibrationData(calibration);
  grabber.start();

  Mat1f pt = rgb_pose.cvProjectionMatrix();
  pt.resize(4);
  transpose(pt, pt);
  ntk_dbg_print(pt, 1);

  //depth_pose->invert();
  Mat1f ipt = depth_pose.cvProjectionMatrix();
  ipt.resize(4);
  ipt = ipt.inv();
  transpose(ipt, ipt); //i thought this matrix was column major

  cl_ipt.copyRawToDevice(ipt.ptr<float>(), 16);
  cl_pt.copyRawToDevice(pt.ptr<float>(), 16);

  RGBDImage current_frame;
  cv::Mat3b mapped_color;
  while (true)
  {
    ntk::TimeCount tc_loop("loop", 2);

    ntk::TimeCount tc_wait_for_frame("wait for next frame", 2);
    grabber.waitForNextFrame();
    tc_wait_for_frame.stop();

    ntk::TimeCount tc_copy_to("copy to current frame", 2);
    grabber.copyImageTo(current_frame);
    tc_copy_to.stop();

    ntk::TimeCount tc_process("rgbd process", 2);
    processor.processImage(current_frame);
    tc_process.stop();

    ntk_assert(current_frame.calibration(), "Ensure there is calibration data in the image");
    mapped_color.create(current_frame.depth().size());
    mapped_color = Vec3b(0,0,0);

    ntk::TimeCount tc_gpu("GPU unproject", 1);
    cl_kinect_depth.copyRawToDevice(current_frame.depth().ptr<float>(),
                                    640*480);
    cl_kinect_rgb.copyRawToDevice(current_frame.rgb().ptr<uchar>(), 640*480*3);
    cli.queue.finish();
    tc_gpu.stop("(copy to gpu)");
    int ctaSize = 16; // work group size
    kernel.execute(640*480, ctaSize);
    cli.queue.finish();
    tc_gpu.stop("(execute)");

    // cl_kinect.mapToHost(kinect_data.size());
    cl_kinect.copyToHost(kinect_data);
    // cl_kinect.copyRawToHost(kinect_raw_data, 640*480);
    cl_kinect_col.copyToHost(kinect_col);
    cli.queue.finish();
    tc_gpu.stop("(copy to host)");
    tc_gpu.stop();
    ntk_dbg_print(kinect_data[0], 1);
    // ntk_dbg_print(kinect_raw_data[0], 1);
    // cl_kinect.unmapToHost();

    ntk::TimeCount tc_mapping("depth-rgb mapping", 1);
    // equivalent to for(int r = 0; r < im.rows; ++r) for (int c = 0; c < im.cols; ++c)
    for_all_rc(current_frame.depth())
    {
      float depth = current_frame.depth()(r,c);

      // Check for invalid depth.
      if (depth < 1e-5)
        continue;

      // Point in depth image.
      Point3f p_depth (c,r,depth);

      // Point in 3D metric space
      Point3f p3d;
      p3d = current_frame.calibration()->depth_pose->unprojectFromImage(p_depth);
      // Debug output: show p3d if the global debug level is >= 1
      // ntk_dbg_print(p3d, 1);

      if (c == 0 && r == 0)
        ntk_dbg_print(p3d, 1);

      // Point in color image
      Point3f p_rgb;
      p_rgb = current_frame.calibration()->rgb_pose->projectToImage(p3d);
      int r_rgb = p_rgb.y;
      int c_rgb = p_rgb.x;      
      // Check if the pixel coordinates are valid and set the value.
      if (is_yx_in_range(current_frame.rgb(), r_rgb, c_rgb))
        mapped_color(r, c) = current_frame.rgb()(r_rgb, c_rgb);
    }
    tc_mapping.stop();

    //int fps = grabber.frameRate();
    int fps = 1000/tc_loop.elapsedMsecs();
    tc_loop.stop();
    cv::putText(current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", current_frame.rgb());

    // Show depth as normalized gray scale
    imshow_normalized("depth", current_frame.depth());

    // Show color values mapped to depth frame
    imshow("mapped_color", mapped_color);

    cv::waitKey(10);
  }
}
