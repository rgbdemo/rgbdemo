////////////////////////////////////////////////////////////////////////////
//	File:		SiftGPU.cpp
//	Author:		Changchang Wu
//	Description :	Implementation of the SIFTGPU classes.
//					SiftGPU:	The SiftGPU Tool.
//					SiftGPUEX:	SiftGPU + viewer
//					SiftParam:	Sift Parameters
//
//	Copyright (c) 2007, 2010 University of North Carolina at Chapel Hill
//	All Rights Reserved
//
//	Permission to use, copy, modify and distribute this software and its
//	documentation for educational, research and non-profit purposes, without
//	fee, and without a written agreement is hereby granted, provided that the
//	above copyright notice and the following paragraph appear in all copies.
//
//	The University of North Carolina at Chapel Hill make no representations
//	about the suitability of this software for any purpose. It is provided
//	'as is' without express or implied warranty.
//
//	Please send BUG REPORTS to ccwu@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////

#include <ntk/ntk.h>

#include "GL/glew.h"
#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
using namespace std;


#include "GlobalUtil.h"
#include "GLTexImage.h"
#include "ShaderMan.h"
#include "FrameBufferObject.h"
#include "SiftPyramid.h"
#include "PyramidGL.h"
#include "SiftGPU.h"

//CUDA works only with vc8 or higher
#if defined(CUDA_SIFTGPU_ENABLED)
#include "PyramidCU.h"
#endif


////
#if  defined(_WIN32)
  #include "direct.h"
  // #pragma warning (disable : 4786)
  // #pragma warning (disable : 4996)
#else
  //compatible with linux
  #define _stricmp strcasecmp
#endif

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//
//just want to make this class invisible
class ImageList:public std::vector<std::string> {};

SiftGPU::SiftGPU(int np)
{
  _texImage = new GLTexInput;
  _imgpath[0] = 0;
  _outpath[0] = 0;
  _initialized = 0;
  _image_loaded = 0;
   GlobalUtil::_UseSiftGPUEX = 0;
  _current = 0;
  _list = new ImageList();

  _nPyramid = np < 1? 1 : np;
  _pyramids = NULL;
  _pyramid = NULL;
}


void* SiftGPU::operator new (size_t  size){
  void * p = malloc(size);
  if (p == 0)
  {
    const std::bad_alloc ba;
    throw ba;
  }
  return p;
}

void SiftGPU::SetActivePyramid(int index)
{
  if(index >=0 && index < _nPyramid)
  {
    _pyramid = _pyramids[index];
  }
}

SiftGPU::~SiftGPU()
{
  if(_pyramids)
  {
    for(int i = 0; i < _nPyramid; i++)
    {
      delete _pyramids[i];
    }
    delete _pyramids;
  }
  delete _texImage;
  delete _list;

  if(_initialized) ShaderMan::DestroyShaders(_sigma_num);

  //Calling glutDestroyWindow function will somehow give a heap corruption
  //if(_glut_id >0) glutDestroyWindow(_glut_id);
}


inline void SiftGPU::InitSiftGPU()
{
  if(_initialized || GlobalUtil::_GoodOpenGL ==0) return;
#if !defined(CUDA_SIFTGPU_ENABLED)
  if(GlobalUtil::_UseCUDA)
  {
    GlobalUtil::_UseCUDA = 0;
    std::cerr	<< "---------------------------------------------------------------------------\n"
          << "CUDA not supported in this binary! To enable it, please use SiftGPU_CUDA_Enable\n"
          << "Project for VS2005 or set siftgpu_enable_cuda to 1 in makefile\n"
          << "----------------------------------------------------------------------------\n";
  }
#else
    if(GlobalUtil::_UseCUDA &&  !PyramidCU::CheckCudaDevice(GlobalUtil::_CudaDeviceIndex))
    {
        std::cerr << "Switch from CUDA to OpenGL\n";
        GlobalUtil::_UseCUDA = 0;
    }
#endif

#if !defined(CG_SIFTGPU_ENABLED)
  if(GlobalUtil::_UseGLSL == 0)
  {
    GlobalUtil::_UseGLSL = 1;
    std::cerr	<< "---------------------------------------------------------------------------\n"
          << "CG not supported in this binary! please define CG_SIFTGPU_ENABLED to enable.\n"
          << "----------------------------------------------------------------------------\n";
  }
#endif

  _pyramids = new SiftPyramid*[_nPyramid];
  for(int i = 0; i < _nPyramid; i++)
  {
#if defined(CUDA_SIFTGPU_ENABLED)
    if(GlobalUtil::_UseCUDA)
      _pyramids[i] = new PyramidCU(*this);
    else
#endif
    if(GlobalUtil::_usePackedTex)
      _pyramids[i] = new PyramidPacked(*this);
    else
      _pyramids[i] = new PyramidNaive(*this);
  }
  _pyramid =  _pyramids[0];

  //Initialize OpenGL parameters
    GlobalUtil::InitGLParam(GlobalUtil::_UseCUDA);

  //sift parameters
  ParseSiftParam();

  if(GlobalUtil::_GoodOpenGL)
  {
    if(GlobalUtil::_verbose)	std::cout<<"\n[SiftGPU Language]:\t"<<
    (GlobalUtil::_UseCUDA? "CUDA" : (GlobalUtil::_UseGLSL?"GLSL" : "CG")) <<"\n\n";

    //Load Global OpenGl shaders..
    if(!GlobalUtil::_UseCUDA)
    {
      GlobalUtil::StartTimer("Load OpenGL Shaders");
      ShaderMan::InitShaderMan();
      ShaderMan::LoadDogShaders(_dog_threshold, _edge_threshold);
      ShaderMan::LoadGenListShader(_dog_level_num, 0);
      ShaderMan::CreateGaussianFilters(*this);
      GlobalUtil::StopTimer();
    }

    if(GlobalUtil::_InitPyramidWidth >0 && GlobalUtil::_InitPyramidHeight >0)
    {
      GlobalUtil::StartTimer("Initialize Pyramids");
      for(int i = 0; i < _nPyramid; i++)
      {
        _pyramid[i].InitPyramid(GlobalUtil::_InitPyramidWidth,
                    GlobalUtil::_InitPyramidHeight, 0);
      }
      GlobalUtil::StopTimer();
    }
  }

  ClockTimer::InitHighResolution();

  _initialized = 1;
}

int	 SiftGPU::RunSIFT(int index)
{
  if(_list->size()>0 )
  {
    index = index % _list->size();
    if(strcmp(_imgpath, _list->at(index).data()))
    {
      strcpy(_imgpath, _list->at(index).data());
      _image_loaded = 0;
      _current = index;
    }
    return RunSIFT();
  }else
  {
    return 0;
  }

}

int  SiftGPU::RunSIFT( int width,  int height, const void * data, unsigned int gl_format, unsigned int gl_type)
{

  if(GlobalUtil::_GoodOpenGL ==0 ) return 0;
  if(!_initialized) InitSiftGPU();
  else GlobalUtil::SetGLParam();
  if(GlobalUtil::_GoodOpenGL ==0 ) return 0;

  if(width > 0 && height >0 && data != NULL)
  {
    _imgpath[0] = 0;
    //try downsample the image on CPU
    GlobalUtil::StartTimer("Upload Image data");
    if(_texImage->SetImageData(width, height, data, gl_format, gl_type))
    {
      _image_loaded = 2; //gldata;
      GlobalUtil::StopTimer();
      _timing[0] = GlobalUtil::GetElapsedTime();

      //if the size of image is different, the pyramid need to be reallocated.
      GlobalUtil::StartTimer("Initialize Pyramid");
      _pyramid->InitPyramid(width, height, _texImage->_down_sampled);
      GlobalUtil::StopTimer();
      _timing[1] = GlobalUtil::GetElapsedTime();

      return RunSIFT();
    }else
    {
      return 0;
    }
  }else
  {
    return 0;
  }

}

int  SiftGPU::RunSIFT(const char * imgpath)
{
  _image_loaded = 0;
  cv::Mat3b color_im = cv::imread(imgpath, 1);
  cv::cvtColor(color_im, _image, CV_BGR2GRAY);
  return RunSIFT();
}

int  SiftGPU::RunSIFT(const cv::Mat1b& image)
{
  _image_loaded = 0;
  _image.create(image.size());
  image.copyTo(_image);
  return RunSIFT();
}

int SiftGPU::RunSIFT(int num, const SiftKeypoint * keys, int keys_have_orientation)
{
  if(num <=0) return 0;
  _pyramid->SetKeypointList(num, (const float*) keys, 1, keys_have_orientation);
  return RunSIFT();
}

int SiftGPU::RunSIFT()
{
  //check image data
  if(_image.data==0 && _image_loaded == 0) return 0;

  //check OpenGL support
  if(GlobalUtil::_GoodOpenGL ==0 ) return 0;

  ClockTimer timer;

  if(!_initialized)
  {
      //initialize SIFT GPU for once
    InitSiftGPU();
    if(GlobalUtil::_GoodOpenGL ==0 ) return 0;
  }else
  {
    //in case some OpenGL parameters are changed by users
    GlobalUtil::SetGLParam();
  }

  timer.StartTimer("RUN SIFT");
  //process input image file
  if( _image_loaded ==0)
  {
    int width, height;
    //load and try down-sample on cpu
    GlobalUtil::StartTimer("Load Input Image");
    if(!_texImage->LoadImageFile(_image, width, height)) return 0;
    _image_loaded = 1;
    GlobalUtil::StopTimer();
    _timing[0] = GlobalUtil::GetElapsedTime();

    //make sure the pyrmid can hold the new image.
    GlobalUtil::StartTimer("Initialize Pyramid");
    _pyramid->InitPyramid(width, height, _texImage->_down_sampled);
    GlobalUtil::StopTimer();
    _timing[1] = GlobalUtil::GetElapsedTime();

  }else
  {
    //change some global states
    if(!GlobalUtil::_UseCUDA)
    {
      GlobalUtil::FitViewPort(1,1);
      _texImage->FitTexViewPort();
    }
    if(_image_loaded == 1)
    {
      _timing[0] = _timing[1] = 0;
    }else
    {//2
      _image_loaded = 1;
    }
  }

  if(_pyramid->_allocated ==0 ) return 0;


#ifdef DEBUG_SIFTGPU
  _pyramid->BeginDEBUG(_imgpath);
#endif

  //process the image
  _pyramid->RunSIFT(_texImage);

    //read back the timing
  _pyramid->GetPyramidTiming(_timing + 2);

  //write output once if there is only one input
  if(_outpath[0] ){SaveSIFT(_outpath);	_outpath[0] = 0;}

  //terminate the process when -exit is provided.
  if(GlobalUtil::_ExitAfterSIFT && GlobalUtil::_UseSiftGPUEX) exit(0);

  timer.StopTimer();
  if(GlobalUtil::_verbose)std::cout<<endl;

    return _pyramid->GetSucessStatus();
}


void SiftGPU::SetKeypointList(int num, const SiftKeypoint * keys, int keys_have_orientation)
{
  _pyramid->SetKeypointList(num, (const float*)keys, 0, keys_have_orientation);
}

void SiftGPU::SetVerbose(int verbose)
{
  GlobalUtil::_timingO = verbose>2;
  GlobalUtil::_timingL = verbose>3;
  if(verbose == -1)
  {
    //Loop between verbose level 0, 1, 2
    if(GlobalUtil::_verbose)
    {
      GlobalUtil::_verbose  = GlobalUtil::_timingS;
      GlobalUtil::_timingS = 0;
      if(GlobalUtil::_verbose ==0 && GlobalUtil::_UseSiftGPUEX)
        std::cout << "Console ouput disabled, press Q/V to enable\n\n";
    }else
    {
      GlobalUtil::_verbose = 1;
      GlobalUtil::_timingS = 1;
    }
  }else if(verbose == -2)
  {
    //trick for disabling all output (still keeps the timing level)
    GlobalUtil::_verbose = 0;
    GlobalUtil::_timingS = 1;
  }else
  {
    GlobalUtil::_verbose = verbose>0;
    GlobalUtil::_timingS = verbose>1;
  }

}


SiftParam::SiftParam()
{

  _level_min = -1;
  _dog_level_num  = 3;
  _level_max = 0;
  _sigma0 = 0;
  _sigman = 0;
  _edge_threshold = 0;
  _dog_threshold =  0;


}

float SiftParam::GetInitialSmoothSigma(int octave_min)
{
  float	sa = _sigma0 * powf(2.0f, float(_level_min)/float(_dog_level_num)) ;
  float   sb = _sigman / powf(2.0f,  float(octave_min)) ;//
  float sigma_skip0 = sa>sb+ 0.001?sqrt(sa*sa - sb*sb): 0.0f;
  return sigma_skip0;
}

void SiftParam::ParseSiftParam()
{

  if(_dog_level_num ==0) _dog_level_num = 3;
  if(_level_max ==0) _level_max = _dog_level_num + 1;
  if(_sigma0 ==0.0f) _sigma0 = 1.6f * powf(2.0f, 1.0f / _dog_level_num) ;
  if(_sigman == 0.0f) _sigman = 0.5f;


  _level_num = _level_max -_level_min + 1;

  _level_ds  = _level_min + _dog_level_num;
  if(_level_ds > _level_max ) _level_ds = _level_max ;


  ///
  float _sigmak = powf(2.0f, 1.0f / _dog_level_num) ;
  float dsigma0 = _sigma0 * sqrt (1.0f - 1.0f / (_sigmak*_sigmak) ) ;
  float sa, sb;


  sa = _sigma0 * powf(_sigmak, (float)_level_min) ;
  sb = _sigman / powf(2.0f,   (float)GlobalUtil::_octave_min_default) ;//

  _sigma_skip0 = sa>sb+ 0.001?sqrt(sa*sa - sb*sb): 0.0f;

    sa = _sigma0 * powf(_sigmak, float(_level_min )) ;
    sb = _sigma0 * powf(_sigmak, float(_level_ds - _dog_level_num)) ;

  _sigma_skip1 = sa>sb + 0.001? sqrt(sa*sa - sb*sb): 0.0f;

  _sigma_num = _level_max - _level_min;
  _sigma = new float[_sigma_num];

  for(int i = _level_min + 1; i <= _level_max; i++)
  {
    _sigma[i-_level_min -1] =  dsigma0 * powf(_sigmak, float(i)) ;
  }

  if(_dog_threshold ==0)	_dog_threshold      = 0.02f / _dog_level_num ;
  if(_edge_threshold==0) _edge_threshold		= 10.0f;
}

void SiftGPU::PrintUsage()
{
  std::cout
  <<"SiftGPU Usage:\n"
  <<"-h -help          : Parameter information\n"
  <<"-i <strings>      : Filename(s) of the input image(s)\n"
  <<"-il <string>      : Filename of an image list file\n"
  <<"-o <string>       : Where to save SIFT features\n"
  <<"-f <float>        : Filter width factor; Width will be 2*factor+1 (default : 4.0)\n"
  <<"-w  <float>       : Orientation sample window factor (default: 2.0)\n"
  <<"-dw <float>  *    : Descriptor grid size factor (default : 3.0)\n"
  <<"-fo <int>    *    : First octave to detect DOG keypoints(default : 0)\n"
  <<"-no <int>         : Maximum number of Octaves (default : no limit)\n"
  <<"-d <int>          : Number of DOG levels in an octave (default : 3)\n"
  <<"-t <float>        : DOG threshold (default : 0.02/3)\n"
  <<"-e <float>        : Edge Threshold (default : 10.0)\n"
  <<"-m  <int=2>       : Multi Feature Orientations (default : 1)\n"
  <<"-m2p              : 2 Orientations packed as one float\n"
  <<"-s  <int=1>       : Sub-Pixel, Sub-Scale Localization, Multi-Refinement(num)\n"
  <<"-lcpu -lc <int>   : CPU/GPU mixed Feature List Generation (defaut : 6)\n"
  <<"                    Use GPU first, and use CPU when reduction size <= pow(2,num)\n"
  <<"                    When <num> is missing or equals -1, no GPU will be used\n"
  <<"-noprep           : Upload raw data to GPU (default: RGB->LUM and down-sample on CPU)\n"
  <<"-sd               : Skip descriptor computation if specified\n"
  <<"-unn    *         : Write unnormalized descriptor if specified\n"
  <<"-b      *         : Write binary sift file if specified\n"
  <<"-fs <int>         : Block Size for freature storage <default : 4>\n"
  <<"-cg               : Use CG SiftGPU instead of GLSL (default : GLSL)\n"
    <<"-cuda <int=0>     : Use CUDA SiftGPU, and specifiy the device index\n"
  <<"-tight            : Automatically resize pyramid to fit new images tightly\n"
  <<"-p  <W>x<H>       : Inititialize the pyramids to contain image of WxH (eg -p 1024x768)\n"
  <<"-lm  <int>        : Maximum feature count for a level (for pre-allocation)\n"
  <<"-lmp <float>      : Maximum percent of pixels as features (for pre-allocaton)\n"
  <<"-tc[1|2|3] <int> *: Threshold for limiting the overall number of features (3 methods)\n"
  <<"-v <int>          : Level of timing details. Same as calling Setverbose() function\n"
  <<"-loweo            : (0, 0) at center of top-left pixel (defaut: corner)\n"
  <<"-maxd <int> *     : Max working dimension (default : 2560 (unpacked) / 3200 (packed))\n"
  <<"-exit             : Exit program after processing the input image\n"
  <<"-unpack           : Use the old unpacked implementation\n"
  <<"-di               : Use dynamic array indexing if available (defualt : no)\n"
  <<"                    It could make computation faster on cards like GTX 280\n"
  <<"-fastmath         : specify -fastmath to cg compiler (Not much difference.)\n"
  <<"-ofix     *       : use 0 as feature orientations.\n"
  <<"-ofix-not *       : disable -ofix.\n"
  <<"-winpos <X>x<Y> * : Screen coordinate used in Win32 to select monitor/GPU.\n"
    <<"-display <string>*: Display name used in Linux/Mac to select monitor/GPU.\n"
    <<"\n"
    <<"NOTE: parameters marked with * can be changed after initialization\n"
  <<"\n";
}

void SiftGPU::ParseParam(int argc, const char **argv)
{
    #define CHAR1_TO_INT(x)         ((x >= 'A' && x <= 'Z') ? x + 32 : x)
    #define CHAR2_TO_INT(str, i)    (str[i] ? CHAR1_TO_INT(str[i]) + (CHAR1_TO_INT(str[i+1]) << 8) : 0)
    #define CHAR3_TO_INT(str, i)    (str[i] ? CHAR1_TO_INT(str[i]) + (CHAR2_TO_INT(str, i + 1) << 8) : 0)
    #define STRING_TO_INT(str)      (CHAR1_TO_INT(str[0]) +  (CHAR3_TO_INT(str, 1) << 8))

#ifdef _MSC_VER
    //charizing is microsoft only
    #define MAKEINT1(a)             (#@a )
#else
    #define mychar0    '0'
    #define mychar1    '1'
    #define mychar2    '2'
    #define mychar3    '3'
    #define mychara    'a'
    #define mycharb    'b'
    #define mycharc    'c'
    #define mychard    'd'
    #define mychare    'e'
    #define mycharf    'f'
    #define mycharg    'g'
    #define mycharh    'h'
    #define mychari    'i'
    #define mycharj    'j'
    #define mychark    'k'
    #define mycharl    'l'
    #define mycharm    'm'
    #define mycharn    'n'
    #define mycharo    'o'
    #define mycharp    'p'
    #define mycharq    'q'
    #define mycharr    'r'
    #define mychars    's'
    #define mychart    't'
    #define mycharu    'u'
    #define mycharv    'v'
    #define mycharw    'w'
    #define mycharx    'x'
    #define mychary    'y'
    #define mycharz    'z'
    #define MAKEINT1(a)             (mychar##a )
#endif
    #define MAKEINT2(a, b)          (MAKEINT1(a) + (MAKEINT1(b) << 8))
    #define MAKEINT3(a, b, c)       (MAKEINT1(a) + (MAKEINT2(b, c) << 8))
    #define MAKEINT4(a, b, c, d)    (MAKEINT1(a) + (MAKEINT3(b, c, d) << 8))


  const char* arg, *param, * opt;
  int  setMaxD = 0, opti;
  for(int i = 0; i< argc; i++)
  {
    arg = argv[i];
    if(arg == NULL || arg[0] != '-' || !arg[1])continue;
    opt = arg+1;
        opti = STRING_TO_INT(opt);
    param = argv[i+1];

        ////////////////////////////////
        switch(opti)
        {
        case MAKEINT1(h):
        case MAKEINT4(h, e, l, p):
      PrintUsage();
            break;
        case MAKEINT4(g, l, s, l):
      if(!_initialized) GlobalUtil::_UseGLSL = 1;
            break;
#ifdef CG_SIFTGPU_ENABLED
        case MAKEINT2(c, g):
      if(!_initialized) GlobalUtil::_UseGLSL = 0;
            break;
#endif
#ifdef CUDA_SIFTGPU_ENABLED
        case MAKEINT4(c, u, d, a):
            if(!_initialized)
            {
          GlobalUtil::_UseCUDA = 1;
                int device =  -1;
          if(i+1 <argc && sscanf(param, "%d", &device) && device >=0)
                {
                    GlobalUtil::_CudaDeviceIndex = device;
                    i++;
                }
            }
            break;
#endif
        case MAKEINT4(p, a, c, k):
      if(!_initialized) GlobalUtil::_usePackedTex = 1;
            break;
        case MAKEINT4(u, n, p, a): //unpack
      if(!_initialized)
            {
                GlobalUtil::_usePackedTex = 0;
                if(!setMaxD) GlobalUtil::_texMaxDim = 2560;
            }
            break;
        case MAKEINT4(l, c, p, u):
        case MAKEINT2(l, c):
            if(!_initialized)
            {
          int gskip = -1;
          if(i+1 <argc)	sscanf(param, "%d", &gskip);
          if(gskip >= 0)
          {
            GlobalUtil::_ListGenSkipGPU = gskip;
          }else
          {
            GlobalUtil::_ListGenGPU = 0;
          }
            }
            break;
        case MAKEINT4(p, r, e, p):
      GlobalUtil::_PreProcessOnCPU = 1;
            break;
        case MAKEINT4(n, o, p, r): //noprep
      GlobalUtil::_PreProcessOnCPU = 0;
            break;
        case MAKEINT4(f, b, o, 1):
      FrameBufferObject::UseSingleFBO =1;
            break;
        case MAKEINT4(f, b, o, s):
      FrameBufferObject::UseSingleFBO = 0;
            break;
    case MAKEINT2(s, d):
      if(!_initialized) GlobalUtil::_DescriptorPPT =0;
            break;
        case MAKEINT3(u, n, n):
      GlobalUtil::_NormalizedSIFT =0;
            break;
        case MAKEINT4(n, d, e, s):
      GlobalUtil::_NormalizedSIFT =1;
            break;
        case MAKEINT1(b):
      GlobalUtil::_BinarySIFT = 1;
            break;
        case MAKEINT4(t, i, g, h): //tight
      GlobalUtil::_ForceTightPyramid = 1;
            break;
        case MAKEINT4(e, x, i, t):
      GlobalUtil::_ExitAfterSIFT = 1;
            break;
        case MAKEINT2(d, i):
      GlobalUtil::_UseDynamicIndexing = 1;
            break;
        case MAKEINT4(s, i, g, n):
            if(!_initialized || GlobalUtil::_UseCUDA) GlobalUtil::_KeepExtremumSign = 1;
            break;
        case MAKEINT4(n, o, g, l):
            GlobalUtil::_NoOpenGLinCUDA = 1;
            break;
    case MAKEINT1(m):
        case MAKEINT2(m, o):
            if(!_initialized)
            {
          int mo = 2; //default multi-orientation
          if(i+1 <argc)	sscanf(param, "%d", &mo);
          //at least two orientation
          GlobalUtil::_MaxOrientation = min(max(1, mo), 4);
            }
            break;
        case MAKEINT3(m, 2, p):
            if(!_initialized)
            {
          GlobalUtil::_MaxOrientation = 2;
          GlobalUtil::_OrientationPack2 = 1;
            }
            break;
        case MAKEINT1(s):
            if(!_initialized)
            {
                int sp = 1; //default refinement
                if(i+1 <argc)	sscanf(param, "%d", &sp);
                //at least two orientation
                GlobalUtil::_SubpixelLocalization = min(max(0, sp),5);
            }
            break;
        case MAKEINT4(o, f, i, x):
      GlobalUtil::_FixedOrientation = (_stricmp(opt, "ofix")==0);
            break;
    case MAKEINT4(l, o, w, e): // loweo
      GlobalUtil::_LoweOrigin = 1;
            break;
        case MAKEINT4(f, a, s, t): //fastmath
      GlobalUtil::_UseFastMath = 1;
            break;
        case MAKEINT4(n, a, r, r): // narrow
      GlobalUtil::_NarrowFeatureTex = 1;
            break;
        case MAKEINT4(d, e, b, u): // debug
      GlobalUtil::_debug = 1;
            break;
        case MAKEINT2(k, 0):
            GlobalUtil::_KeyPointListForceLevel0 = 1;
            break;
        case MAKEINT2(k, x):
            GlobalUtil::_KeyPointListForceLevel0 = 0;
            break;
        default:
            if(i + 1 >= argc) break;
            switch(opti)
            {
            case MAKEINT1(i):
                strcpy(_imgpath, param);
                i++;
                //get the file list..
                _list->push_back(param);
                while( i+1 < argc && argv[i+1][0] !='-')
                {
                    _list->push_back(argv[++i]);
                }
                break;
            case MAKEINT2(i, l):
                LoadImageList(param);
                i++;
                break;
            case MAKEINT1(o):
                strcpy(_outpath, param);
                i++;
                break;
            case MAKEINT1(f):
                {
                    float factor = 0.0f;
                    if(sscanf(param, "%f", &factor) && factor > 0 )
                    {
                        GlobalUtil::_FilterWidthFactor  = factor;
                        i++;
                    }
                }
                break;
            case MAKEINT2(o, t):
                {
                    float factor = 0.0f;
                    if(sscanf(param, "%f", &factor) && factor>0 )
                    {
                        GlobalUtil::_MulitiOrientationThreshold  = factor;
                        i++;
                    }
                    break;
                }
            case MAKEINT1(w):
                {
                    float factor = 0.0f;
                    if(sscanf(param, "%f", &factor) && factor>0 )
                    {
                        GlobalUtil::_OrientationWindowFactor  = factor;
                        i++;
                    }
                    break;
                }
            case MAKEINT2(d, w):
                {
                    float factor = 0.0f;
                    if(sscanf(param, "%f", &factor) && factor > 0 )
                    {
                        GlobalUtil::_DescriptorWindowFactor  = factor;
                        i++;
                    }
                    break;
                }
            case MAKEINT2(f, o):
                {
                    int first_octave = -3;
                    if(sscanf(param, "%d", &first_octave) && first_octave >=-2 )
                    {
                        GlobalUtil::_octave_min_default = first_octave;
                        i++;
                    }
                    break;
                }
            case MAKEINT2(n, o):
                if(!_initialized)
                {
                    int octave_num=-1;
                    if(sscanf(param, "%d", &octave_num))
                    {
                        octave_num = max(-1, octave_num);
                        if(octave_num ==-1 || octave_num >=1)
                        {
                            GlobalUtil::_octave_num_default = octave_num;
                            i++;
                        }
                    }
                }
                break;
            case MAKEINT1(t):
                {
                    float threshold = 0.0f;
                    if(sscanf(param, "%f", &threshold) && threshold >0 && threshold < 0.5f)
                    {
                        SiftParam::_dog_threshold = threshold;
                        i++;
                    }
                    break;
                }
            case MAKEINT1(e):
                {
                    float threshold = 0.0f;
                    if(sscanf(param, "%f", &threshold) && threshold >0 )
                    {
                        SiftParam::_edge_threshold = threshold;
                        i++;
                    }
                    break;
                }
            case MAKEINT1(d):
                {
                    int num = 0;
                    if(sscanf(param, "%d", &num) && num >=1 && num <=10)
                    {
                        SiftParam::_dog_level_num = num;
                        i++;
                    }
                    break;
                }
            case MAKEINT2(f, s):
                {
                    int num = 0;
                    if(sscanf(param, "%d", &num) && num >=1)
                    {
                        GlobalParam::_FeatureTexBlock = num;
                        i++;
                    }
                    break;
                }
            case MAKEINT1(p):
                {
                    int w =0, h=0;
                    if(sscanf(param, "%dx%d", &w, &h) == 2 && w >0 &&  h>0)
                    {
                        GlobalParam::_InitPyramidWidth = w;
                        GlobalParam::_InitPyramidHeight = h;
                        i++;
                    }
                    break;
                }
            case MAKEINT4(w, i, n, p): //winpos
                {
                    int x =0, y=0;
                    if(sscanf(param, "%dx%d", &x, &y) == 2)
                    {
                        GlobalParam::_WindowInitX = x;
                        GlobalParam::_WindowInitY = y;
                        i++;
                    }
                    break;
                }
            case MAKEINT4(d, i, s, p): //display
                {
                    GlobalParam::_WindowDisplay = param;
                    i++;
                    break;
                }
            case MAKEINT2(l, m):
                {
                    int num = 0;
                    if(sscanf(param, "%d", &num) && num >=1000)
                    {
                        GlobalParam::_MaxLevelFeatureNum = num;
                        i++;
                    }
                    break;
                }
            case MAKEINT3(l, m, p):
                {
                    float num = 0.0f;
                    if(sscanf(param, "%f", &num) && num >=0.001)
                    {
                        GlobalParam::_MaxFeaturePercent = num;
                        i++;
                    }
                    break;
                }
            case MAKEINT3(t, c, 2): //downward
            case MAKEINT3(t, c, 3):
            case MAKEINT2(t, c):  //tc
            case MAKEINT3(t, c, 1):  //
                {
                    switch (opti)
                    {
                        case MAKEINT3(t, c, 2): GlobalUtil::_TruncateMethod = 1; break;
                        case MAKEINT3(t, c, 3): GlobalUtil::_TruncateMethod = 2; break;
                        default:                GlobalUtil::_TruncateMethod = 0; break;
                    }
                    int num = -1;
                    if(sscanf(param, "%d", &num) && num > 0)
                    {
                        GlobalParam::_FeatureCountThreshold = num;
                        i++;
                    }
                    break;
                }


            case MAKEINT1(v):
                {
                    int num = 0;
                    if(sscanf(param, "%d", &num) && num >=0 && num <= 4)
                    {
                        SetVerbose(num);
                    }
                    break;
                }
            case MAKEINT4(m, a, x, d):
                {
                    int num = 0;
                    if(sscanf(param, "%d", &num) && num > 0)
                    {
                        GlobalUtil::_texMaxDim = num;
                        setMaxD = 1;
                    }
                    break;
                }
            default:
                break;
            }
            break;
        }
  }
  //do not write result if there are more than one input images
  if(_outpath[0] && _list->size()>1)		_outpath[0] = 0;
}

void SiftGPU::SetImageList(int nimage, const char** filelist)
{
  _list->resize(0);
  for(int i = 0; i < nimage; i++)
  {
    _list->push_back(filelist[i]);
  }
  _current = 0;

}
void SiftGPU:: LoadImageList(const char *imlist)
{
  char filename[_MAX_PATH];
  ifstream in(imlist);
  while(in>>filename)
  {
    _list->push_back(filename);
  }
  in.close();


  if(_list->size()>0)
  {
    strcpy(_imgpath, _list->at(0).data());
    strcpy(filename, imlist);
    char * slash = strrchr(filename, '\\');
    if(slash == 0) slash = strrchr(filename, '/');
    if(slash )
    {
      slash[1] = 0;
      int ok = chdir(filename);
    }
  }
  _image_loaded = 0;


}
float SiftParam::GetLevelSigma( int lev)
{
  return _sigma0 * powf( 2.0f,  float(lev) / float(_dog_level_num )); //bug fix 9/12/2007
}

int SiftGPU::CreateContextGL()
{
  if(GlobalUtil::_UseCUDA  &&  GlobalUtil::_NoOpenGLinCUDA) {}
  else if(!GlobalUtil::CreateWindowEZ()) return 0;

  return VerifyContextGL();
}

int SiftGPU::VerifyContextGL()
{
  InitSiftGPU();
  return (GlobalUtil::_GoodOpenGL > 0) + GlobalUtil::_FullSupported;
}

int SiftGPU::IsFullSupported()
{
  return GlobalUtil::_GoodOpenGL > 0 &&  GlobalUtil::_FullSupported;
}

void SiftGPU::SaveSIFT(const char * szFileName)
{
  _pyramid->SaveSIFT(szFileName);
}

int SiftGPU::GetFeatureNum()
{
  return _pyramid->GetFeatureNum();
}

void SiftGPU::GetFeatureVector(SiftKeypoint * keys, float * descriptors)
{
//	keys.resize(_pyramid->GetFeatureNum());
  if(GlobalUtil::_DescriptorPPT)
  {
  //	descriptors.resize(128*_pyramid->GetFeatureNum());
    _pyramid->CopyFeatureVector((float*) (&keys[0]), &descriptors[0]);
  }else
  {
    //descriptors.resize(0);
    _pyramid->CopyFeatureVector((float*) (&keys[0]), NULL);
  }
}

void SiftGPU::SetTightPyramid(int tight)
{
  GlobalUtil::_ForceTightPyramid = tight;
}

int SiftGPU::AllocatePyramid(int width, int height)
{
  _pyramid->_down_sample_factor = 0;
  _pyramid->_octave_min = GlobalUtil::_octave_min_default;
  if(GlobalUtil::_octave_min_default>=0)
  {
    width >>= GlobalUtil::_octave_min_default;
    height >>= GlobalUtil::_octave_min_default;
  }else
  {
    width <<= (-GlobalUtil::_octave_min_default);
    height <<= (-GlobalUtil::_octave_min_default);
  }
  _pyramid->ResizePyramid(width, height);
  return _pyramid->_pyramid_height == height && width == _pyramid->_pyramid_width ;
}
void SiftGPU::SetMaxDimension(int sz)
{
  if(sz < GlobalUtil::_texMaxDimGL)
  {
    GlobalUtil::_texMaxDim = sz;
  }
}
int SiftGPU::GetImageCount()
{
  return _list->size();
}

SiftGPU* CreateNewSiftGPU(int np)
{
  return new SiftGPU(np);
}

