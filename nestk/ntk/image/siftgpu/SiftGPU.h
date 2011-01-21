//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef NTK_IMAGE_SIFTGPU_SIFTGPU_H
# define NTK_IMAGE_SIFTGPU_SIFTGPU_H

#include <ntk/core.h>
#include <memory>

#if  defined(_WIN32)
  #ifdef SIFTGPU_DLL
    #ifdef DLL_EXPORT
      #define SIFTGPU_EXPORT __declspec(dllexport)
    #else
      #define SIFTGPU_EXPORT __declspec(dllimport)
    #endif
  #else
    #define SIFTGPU_EXPORT
  #endif

    #define SIFTGPU_EXPORT_EXTERN SIFTGPU_EXPORT

  #if _MSC_VER > 1000
    #pragma once
  #endif
#else
  #define SIFTGPU_EXPORT
    #define SIFTGPU_EXPORT_EXTERN extern "C"
#endif

#if !defined(_MAX_PATH)
  #if defined (MAX_PATH)
    #define _MAX_PATH MAX_PATH
  #else
    #define _MAX_PATH 512
  #endif
#endif

class GLTexInput;
class ShaderMan;
class SiftPyramid;
class ImageList;

  class SiftParam
  {
  public:
    float*		_sigma;
    float		_sigma_skip0; //
    float		_sigma_skip1; //

    //sigma of the first level
    float		_sigma0;
    float		_sigman;
    int			_sigma_num;

    //how many dog_level in an octave
    int			_dog_level_num;
    int			_level_num;

    //starting level in an octave
    int			_level_min;
    int			_level_max;
    int			_level_ds;
    //dog threshold
    float		_dog_threshold;
    //edge elimination
    float		_edge_threshold;
    void		 ParseSiftParam();
  public:
    float GetLevelSigma(int lev);
    float GetInitialSmoothSigma(int octave_min);
    SIFTGPU_EXPORT SiftParam();
  };

  class SiftGPU : public SiftParam
  {
  public:
    enum
    {
      SIFTGPU_NOT_SUPPORTED = 0,
      SIFTGPU_PARTIAL_SUPPORTED = 1, // detction works, but not orientation/descriptor
      SIFTGPU_FULL_SUPPORTED = 2
    };
    typedef struct SiftKeypoint
    {
      float x, y, s, o; //x, y, scale, orientation.
    }SiftKeypoint;
  protected:
    //when more than one images are specified
    //_current indicates the active one
    int		_current;
    //_initialized indicates if the shaders and OpenGL/SIFT parameters are initialized
    //they are initialized only once for one SiftGPU inistance
    //that is, SIFT parameters will not be changed
    int		_initialized;
    //_image_loaded indicates if the current images are loaded
    int		_image_loaded;
    cv::Mat1b _image;
    //the name of current input image
    char	_imgpath[_MAX_PATH];
    //_outpath containes the name of the output file
    char	_outpath[_MAX_PATH];
    //the list of image filenames
    ImageList *    _list;
    //the texture that holds loaded input image
    GLTexInput *   _texImage;
    //the SiftPyramid
    SiftPyramid *  _pyramid;
    SiftPyramid ** _pyramids;
    int			   _nPyramid;

    //print out the command line options
    static void PrintUsage();
    //Initialize OpenGL and SIFT paremeters, and create the shaders accordingly
    void InitSiftGPU();
    //load the image list from a file
    void LoadImageList(const char *imlist);
  public:
    //timing results for 10 steps
    float			    _timing[10];
      inline const char*  GetCurrentImagePath() {return _imgpath; }
  public:
    //set the image list for processing
    SIFTGPU_EXPORT virtual void SetImageList(int nimage, const char** filelist);
    //get the number of SIFT features in current image
    SIFTGPU_EXPORT virtual int	GetFeatureNum();
    //save the SIFT result as a ANSCII/BINARY file
    SIFTGPU_EXPORT virtual void SaveSIFT(const char * szFileName);
    //Copy the SIFT result to two vectors
    SIFTGPU_EXPORT virtual void GetFeatureVector(SiftKeypoint * keys, float * descriptors);
    //Set keypoint list before running sift to get descriptors
    SIFTGPU_EXPORT virtual void SetKeypointList(int num, const SiftKeypoint * keys, int keys_have_orientation = 1);
    //Enable downloading results to CPU.
    //create a new OpenGL context for processing
    //call VerifyContextGL instead if you want to crate openGL context yourself, or your are
    //mixing mixing siftgpu with other openGL code
    SIFTGPU_EXPORT virtual int CreateContextGL();
    //verify the current opengl context..
    //(for example, you call wglmakecurrent yourself and verify the current context)
    SIFTGPU_EXPORT virtual int VerifyContextGL();
    //check if all siftgpu functions are supported
    SIFTGPU_EXPORT virtual int IsFullSupported();
    //set verbose mode
    SIFTGPU_EXPORT virtual void SetVerbose(int verbose = 4);
    //set SiftGPU to brief display mode, which is faster
    inline void SetVerboseBrief(){SetVerbose(2);}
    //parse SiftGPU parameters
    SIFTGPU_EXPORT virtual void ParseParam(int argc, const char **argv);
    //run SIFT on a new image given filename
    SIFTGPU_EXPORT virtual int  RunSIFT(const char * imgpath);
    //run SIFT on a new opencv image
    SIFTGPU_EXPORT virtual int  RunSIFT(const cv::Mat1b& image);
    //run SIFT on an image in the image list given the file index
    SIFTGPU_EXPORT virtual int	RunSIFT(int index);
    //run SIFT on a new image given the pixel data and format/type;
    //gl_format (e.g. GL_LUMINANCE, GL_RGB) is the format of the pixel data
    //gl_type (e.g. GL_UNSIGNED_BYTE, GL_FLOAT) is the data type of the pixel data;
    //Check glTexImage2D(...format, type,...) for the accepted values
    //Using image data of GL_LUMINANCE + GL_UNSIGNED_BYTE can minimize transfer time
    SIFTGPU_EXPORT virtual int  RunSIFT(int width, int height,	const void * data,
                      unsigned int gl_format, unsigned int gl_type);
    //run SIFT on current image (specified by arguments), or processing the current image again
    SIFTGPU_EXPORT virtual int  RunSIFT();
    //run SIFT with keypoints on current image again.
    SIFTGPU_EXPORT virtual int  RunSIFT(int num, const SiftKeypoint * keys, int keys_have_orientation = 1);
    //constructor, (np is the number of pyramids)
    SIFTGPU_EXPORT SiftGPU(int np = 1);
    //destructor
    SIFTGPU_EXPORT virtual ~SiftGPU();
    //set the active pyramid
    SIFTGPU_EXPORT virtual void  SetActivePyramid(int index);
    //retrieve the number of images in the image list
    SIFTGPU_EXPORT virtual int GetImageCount();
    //set parameter GlobalUtil::_ForceTightPyramid
    SIFTGPU_EXPORT virtual void SetTightPyramid(int tight = 1);
    //allocate pyramid for a given size of image
    SIFTGPU_EXPORT virtual int AllocatePyramid(int width, int height);
    //none of the texture in processing can be larger
    //automatic down-sample is used if necessary.
    SIFTGPU_EXPORT virtual void SetMaxDimension(int sz);
    ///
  public:
    //overload the new operator because delete operator is virtual
    //and it is operating on the heap inside the dll (due to the
    //compiler setting of /MT and /MTd). Without the overloaded operator
    //deleting a SiftGPU object will cause a heap corruption in the
    //static link case (but not for the runtime dll loading).
    SIFTGPU_EXPORT void* operator new (size_t size);
  };

#endif // !NTK_IMAGE_SIFTGPU_SIFTGPU_H
