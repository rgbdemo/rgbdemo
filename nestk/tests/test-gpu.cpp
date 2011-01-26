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

__kernel void project( __global float* kinect_3d, // array of float3
                      __global uchar* kinect_rgb_map,
                      __global float* depth,
                      __global uchar* rgb,    //packed as 3 uchar from OpenCV
                      __constant float* rgb_project,
                      __constant float* depth_unproject,
                      int w,
                      int h)
{
  unsigned int i = get_global_id(0);
  float d = depth[i];

  //unproject from depth, in place
  const int c = i % w;
  const int r = (int)(i / w);
  float4 epix = (float4)(d*c, d*r, d, 1);
  float4 p3d = matmult4(depth_unproject, epix);
  kinect_3d[i*3] = p3d.x;
  kinect_3d[i*3+1] = p3d.y;
  kinect_3d[i*3+2] = p3d.z;

  // project it back on the rgb image.
  epix = matmult4(rgb_project, p3d);
  int x = (int)(epix.x/epix.z);
  int y = (int)(epix.y/epix.z);

  if(x>=0 && y>=0 && x < w && y < h)
  {
    int irgb = y*w*3 + x*3;
    kinect_rgb_map[i*3] = rgb[irgb];
    kinect_rgb_map[i*3+1] = rgb[irgb+1];
    kinect_rgb_map[i*3+2] = rgb[irgb+2];
  }
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
  cv::Mat3f kinect_3d(Size(640,480));
  cv::Mat3b kinect_rgb_map(Size(640,480));
  std::vector<float> kinect_trans(16);
  Vec4f* kinect_raw_data = 0;

  // Output buffers.
  Buffer<Vec3f> cl_kinect_3d(&cli, 640*480, BufferBase::WriteOnly);
  Buffer<Vec3b> cl_kinect_rgb_mapped(&cli, 640*480, BufferBase::ReadWrite);

  // Input buffers.
  Buffer<float> cl_kinect_depth(&cli, 640*480, BufferBase::ReadOnly);
  Buffer<Vec3b> cl_kinect_rgb(&cli, 640*480, BufferBase::ReadOnly);

  // projection transforms, 4x4 matrices
  Buffer<float> cl_rgb_project(&cli, 16, BufferBase::ReadOnly);
  Buffer<float> cl_depth_unproject(&cli, 16, BufferBase::ReadOnly);

  Kernel kernel (&cli, "project", project_cl_code);
  {
    int args = 0;
    kernel.setArg(args++, cl_kinect_3d.getDevicePtr());
    kernel.setArg(args++, cl_kinect_rgb_mapped.getDevicePtr());
    kernel.setArg(args++, cl_kinect_depth.getDevicePtr());
    kernel.setArg(args++, cl_kinect_rgb.getDevicePtr());
    kernel.setArg(args++, cl_rgb_project.getDevicePtr());
    kernel.setArg(args++, cl_depth_unproject.getDevicePtr());
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

  Mat1f rgb_project = rgb_pose.cvProjectionMatrix();
  rgb_project.resize(4);
  // column major in opencl
  transpose(rgb_project, rgb_project);

  Mat1f depth_unproject = depth_pose.cvProjectionMatrix();
  depth_unproject.resize(4);
  depth_unproject = depth_unproject.inv();
  // column major in opencl
  transpose(depth_unproject, depth_unproject);

  cl_depth_unproject.copyToDevice(depth_unproject.ptr<float>(), 16);
  cl_rgb_project.copyToDevice(rgb_project.ptr<float>(), 16);

  RGBDImage current_frame;
  int n_frames = 0;
  int accumulated_time = 0;
  int fps = 0;
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
    kinect_rgb_map = Vec3b(0,0,0);

#ifdef USE_OPENCL
    ntk::TimeCount tc_gpu("GPU unproject", 1);
    cl_kinect_depth.copyToDevice(current_frame.depth().ptr<float>(), 640*480);
    cl_kinect_rgb.copyToDevice(current_frame.rgb().ptr<Vec3b>(), 640*480);
    cl_kinect_rgb_mapped.copyToDevice(kinect_rgb_map.ptr<Vec3b>(), kinect_rgb_map.size().area());

    // tc_gpu.stop("(copy to gpu)");
    int ctaSize = 128; // work group size
    kernel.execute(640*480, ctaSize);
    // tc_gpu.stop("(execute)");

    cl_kinect_3d.copyToHost(kinect_3d.ptr<Vec3f>(), kinect_3d.size().area());
    cl_kinect_rgb_mapped.copyToHost(kinect_rgb_map.ptr<Vec3b>(), kinect_rgb_map.size().area());
    cli.queue.finish();
    // tc_gpu.stop("(copy to host)");
    tc_gpu.stop();
#else
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

      // Point in color image
      Point3f p_rgb;
      p_rgb = current_frame.calibration()->rgb_pose->projectToImage(p3d);
      int r_rgb = p_rgb.y;
      int c_rgb = p_rgb.x;      
      // Check if the pixel coordinates are valid and set the value.
      if (is_yx_in_range(current_frame.rgb(), r_rgb, c_rgb))
        kinect_rgb_map(r, c) = current_frame.rgb()(r_rgb, c_rgb);
    }
    tc_mapping.stop();
#endif

    //int fps = grabber.frameRate();
    ++n_frames;
    accumulated_time += tc_loop.elapsedMsecs();
    if (accumulated_time > 1000)
    {
      ntk_dbg_print(accumulated_time, 1);
      ntk_dbg_print(n_frames, 1);
      fps = 1000.0 * n_frames / accumulated_time;
      n_frames = 0;
      accumulated_time = 0;
    }
    tc_loop.stop();
    cv::putText(current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", current_frame.rgb());

    // Show depth as normalized gray scale
    imshow_normalized("depth", current_frame.depth());

    // Show color values mapped to depth frame
    imshow("kinect_rgb_map", kinect_rgb_map);

    cv::waitKey(10);
  }
}
