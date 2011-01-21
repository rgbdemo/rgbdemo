////////////////////////////////////////////////////////////////////////////
//	File:		ProgramGPU.h
//	Author:		Changchang Wu
//	Description : Based class for GPU programs
//		ProgramGPU:	base class of ProgramCG and ProgramGLSL
//		FilterProgram:	base class of FilterGLCG, FilterGLSL, FilterPKCG
//		ShaderBag:	base class of ShaderBagCG, ShaderBagPKCG and ShaderBagGLSL
//
//	Copyright (c) 2007 University of North Carolina at Chapel Hill
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


#ifndef _PROGRAM_GPU_H
#define _PROGRAM_GPU_H

////////////////////////////////////////////////////////////////////////////
//class		ProgramGPU
//description:	pure virtual class
//				provides a common interface of CG and GLSL programs
//				two implementations are ProgramCG and ProgramGLSL
///////////////////////////////////////////////////////////////////////////
class GLTexImage;
class ProgramGPU
{
public:
	//use a gpu program
	virtual int UseProgram() = 0;
	//not used
	virtual ~ProgramGPU(){};
};

///////////////////////////////////////////////////////////////////////////
//class			FilterProgram
//description:	virtual	
//				common interface of CG and GLSL gaussian filters
///////////////////////////////////////////////////////////////////////////
class FilterProgram
{
public:
	ProgramGPU * s_shader_h;
	ProgramGPU * s_shader_v;
	int				_size;
	int				_id; 
	//run filter
	void	RunFilter(GLTexImage* dst, GLTexImage*src, GLTexImage* tmp = NULL);
	//create seperable gaussian filter
	virtual ProgramGPU* CreateFilterH(float kernel[], float offset[], int width)=0;
	virtual ProgramGPU* CreateFilterV(float kernel[], float offset[], int height)=0;
	//create seperable gaussian filter of packed version
	virtual ProgramGPU* CreateFilterHPK(float kernel[], float offset[], int width)=0;
	virtual ProgramGPU* CreateFilterVPK(float kernel[], float offset[], int height)=0;
	//create gaussian filter
	void	CreateGaussianFilter(float sigma);
	FilterProgram()
	{
		s_shader_h = NULL;
		s_shader_v = NULL;
		_size = 0;
		_id = 0; 
	}
	virtual ~FilterProgram()
	{
		if(s_shader_h) delete s_shader_h;
		if(s_shader_v) delete s_shader_v;
	}
};

/////////////////////////////////////////////////////////////////////////////////
//class ShaderBag
//desciption:	pure virtual class
//				provides storage and usage interface of all the shaders for SIFT
//				three implementations are ShaderBagCG, ShaderBagPKCG and ShaderBagGLSL
/////////////////////////////////////////////////////////////////////////////////
class ShaderBag
{
public:
	//shader:	rgb to gray
	ProgramGPU  * s_gray;
	//shader:	copy keypoint to PBO
	ProgramGPU  * s_copy_key;
	//shader:	debug view
	ProgramGPU  * s_debug;
	//shader:	orientation
	//shader:	assign simple orientation to keypoints if hardware is low
	ProgramGPU  * s_orientation;
	//shader:	display gaussian levels
	ProgramGPU  * s_display_gaussian;
	//shader:	display difference of gassian
	ProgramGPU  * s_display_dog;
	//shader:	display  gradient
	ProgramGPU  * s_display_grad;
	//shader:	display keypoints as red(maximum) and blue (minimum)
	ProgramGPU  * s_display_keys;
	//shader:	up/down-sample
	ProgramGPU  * s_sampling;
	//shader:	compute gradient/dog
	ProgramGPU  * s_grad_pass;
	ProgramGPU  * s_dog_pass;
	//shader:   keypoint detection in one pass
	ProgramGPU  * s_keypoint;
	ProgramGPU  * s_seperate_sp;
	//shader:   feature list generations..
	ProgramGPU	* s_genlist_init_tight;
	ProgramGPU	* s_genlist_init_ex;
	ProgramGPU	* s_genlist_histo;
	ProgramGPU	* s_genlist_start;
	ProgramGPU	* s_genlist_step;
	ProgramGPU	* s_genlist_end;
	ProgramGPU	* s_zero_pass;

	//shader:	generate vertex to display SIFT as a square
	ProgramGPU  * s_vertex_list;

	//shader:	descriptor
	ProgramGPU  * s_descriptor_fp;

	//shader:	copy pixels to margin
	ProgramGPU	* s_margin_copy;
public:
	virtual void SetGenListInitParam(int w, int h){};
	virtual void SetGenListEndParam(int ktex){};
	virtual void SetMarginCopyParam(int xmax, int ymax){};
	virtual void LoadDescriptorShader(){};
	virtual void SetFeatureDescirptorParam(int gtex, int otex, float dwidth, float fwidth, float width, float height, float sigma){};
	virtual void SetFeatureOrientationParam(int gtex, int width, int height, float sigma, int stex, float step){};
	virtual void SetSimpleOrientationInput(int oTex, float sigma, float sigma_step){};
	virtual void LoadOrientationShader() =0;
	virtual void SetGenListStartParam(float width, int tex0) =0;
	virtual void LoadGenListShader(int ndoglev, int nlev)=0;
	virtual void UnloadProgram()=0;
	virtual void LoadKeypointShader(float threshold, float edgeTrheshold) = 0;
	virtual int  LoadKeypointShaderMR(float threshold, float edgeThreshold){return 0;}
	virtual void LoadFixedShaders()=0;
	virtual void LoadDisplayShaders() = 0;
	virtual void SetDogTexParam(int texU, int texD)=0;
	virtual void SetGradPassParam(int texP=0){}
	virtual void SetGenListStepParam(int tex, int tex0) = 0;
	virtual void SetGenVBOParam( float width, float fwidth, float size)=0;
	ShaderBag()
	{
		s_debug = 0;
		s_orientation = 0;
		s_display_gaussian = 0;
		s_display_dog = 0;
		s_display_grad = 0;
		s_display_keys = 0;
		s_sampling = 0;
		s_grad_pass = 0;
		s_dog_pass = 0;
		s_keypoint = 0;
		s_seperate_sp = 0;
		s_genlist_init_tight = 0;
		s_genlist_init_ex = 0;
		s_genlist_histo = 0;
		s_genlist_start = 0;
		s_genlist_step = 0;
		s_genlist_end = 0;
		s_vertex_list = 0;
		s_descriptor_fp = 0;
		s_margin_copy = 0;

	}
	virtual ~ShaderBag(); 

};
#endif

