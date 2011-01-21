////////////////////////////////////////////////////////////////////////////
//	File:		ProgramGLSL.h
//	Author:		Changchang Wu
//	Description : Interface for ProgramGLSL classes
//		ProgramGLSL:	Glsl Program
//		FilterGLSL:		Glsl Gaussian Filters
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


#ifndef _PROGRAM_GLSL_H
#define _PROGRAM_GLSL_H


#include "ProgramGPU.h"

class ProgramGLSL:public ProgramGPU
{
	class ShaderObject
	{
		GLuint			_shaderID;
		int				_type;
		int			_compiled;

		static int ReadShaderFile(const char * source,  char *& code);	
		void CheckCompileLog();
	public:
		void PrintCompileLog(ostream & os  );
		int inline IsValidShaderObject(){	return _shaderID && _compiled;}
		int IsValidVertexShader();
		int IsValidFragmentShader();
		GLuint GetShaderID(){return _shaderID;}
		~ShaderObject(); 
		ShaderObject(int shadertype,  const char * source, int filesource =0);
	};


protected:
	int			_linked;
	GLuint		_programID; 
private:
	void AttachShaderObject(ShaderObject& shader);
	void DetachShaderObject(ShaderObject& shader);

public:
	void ReLink();
	int IsNative();
	GLuint GetProgramID(){return _programID;}
	int	UseProgram();
	void PrintLinkLog(std::ostream&os);
	int ValidateProgram();
	void CheckLinkLog();
	int LinkProgram();
	operator GLuint (){return _programID;}
public:
	ProgramGLSL();
	~ProgramGLSL();
	ProgramGLSL(const char* frag_source);
	//ProgramGLSL(char*frag_source, char * vert_source );
};



class GLTexImage;

class FilterGLSL: public FilterProgram
{
private:

	virtual ProgramGPU* CreateFilterH(float kernel[], float offset[], int width);
	virtual ProgramGPU* CreateFilterV(float kernel[], float offset[], int height);
	//packed version
	ProgramGPU* CreateFilterHPK(float kernel[], float offset[], int width);
	ProgramGPU* CreateFilterVPK(float kernel[], float offset[],int height);
};

class ShaderBagGLSL:public ShaderBag
{
	GLint _param_dog_texu;
	GLint _param_dog_texd;
	GLint _param_ftex_width;
	GLint _param_genlist_start_tex0;
	GLint _param_genlist_step_tex;
	GLint _param_genlist_step_tex0;
	GLint _param_genvbo_size;
	GLint _param_orientation_gtex;
	GLint _param_orientation_size;
	GLint _param_orientation_stex;
	GLint _param_margin_copy_truncate;
	GLint _param_genlist_init_bbox;
	GLint _param_descriptor_gtex;
	GLint _param_descriptor_size;
	GLint _param_descriptor_dsize;
public:
	virtual void SetMarginCopyParam(int xmax, int ymax);
	void SetSimpleOrientationInput(int oTex, float sigma, float sigma_step);
	void LoadOrientationShader();
	void LoadDescriptorShaderF2();
	virtual void LoadDescriptorShader();
	virtual void SetFeatureOrientationParam(int gtex, int width, int height, float sigma, int stex = 0, float step = 1.0f);
	virtual void SetFeatureDescirptorParam(int gtex, int otex, float dwidth, float fwidth, float width, float height, float sigma);	
	static void  WriteOrientationCodeToStream(ostream& out);
	static ProgramGLSL* LoadGenListStepShader(int start, int step);
	virtual void SetGenListInitParam(int w, int h);
	virtual void SetGenListStartParam(float width, int tex0);
	virtual void LoadGenListShader(int ndoglev, int nlev);
	virtual void UnloadProgram();
	virtual void LoadKeypointShader(float threshold, float edgeTrheshold);
	virtual void LoadFixedShaders();
	virtual void LoadDisplayShaders();
	virtual void SetDogTexParam(int texU, int texD);
	virtual void SetGenListStepParam(int tex, int tex0);
	virtual void SetGenVBOParam( float width, float fwidth, float size);
	virtual ~ShaderBagGLSL(){}
};


class ShaderBagPKSL:public ShaderBag
{
private:
	GLint	_param_dog_texu;
	GLint	_param_dog_texd;
	GLint	_param_margin_copy_truncate;
	GLint	_param_grad_pass_texp;
	GLint	_param_genlist_init_bbox;
	GLint	_param_genlist_start_tex0;
	GLint	_param_ftex_width;
	GLint	_param_genlist_step_tex;
	GLint	_param_genlist_step_tex0;
	GLint	_param_genlist_end_ktex;
	GLint	_param_genvbo_size;
	GLint	_param_orientation_gtex;
	GLint	_param_orientation_otex;
	GLint	_param_orientation_size;
	GLint	_param_descriptor_gtex; 
	GLint	_param_descriptor_otex;
	GLint	_param_descriptor_size; 
	GLint	_param_descriptor_dsize;

    //
    ProgramGLSL* s_rect_description; 
public:
    ShaderBagPKSL () {s_rect_description = NULL; }
	virtual ~ShaderBagPKSL() {if(s_rect_description) delete s_rect_description; }
	virtual void LoadFixedShaders();
	virtual void LoadDisplayShaders();
	virtual void LoadOrientationShader() ;
	virtual void SetGenListStartParam(float width, int tex0) ;
	virtual void LoadGenListShader(int ndoglev, int nlev);
	virtual void UnloadProgram();
	virtual void LoadKeypointShader(float threshold, float edgeTrheshold) ;
	virtual void LoadDescriptorShader();
	virtual void LoadDescriptorShaderF2();
    static ProgramGLSL* LoadDescriptorProgramRECT();
	static ProgramGLSL* LoadDescriptorProgramPKSL();
/////////////////
	virtual void SetDogTexParam(int texU, int texD);
	virtual void SetGradPassParam(int texP);
	virtual void SetGenListStepParam(int tex, int tex0);
	virtual void SetGenVBOParam( float width, float fwidth, float size);
	virtual void SetFeatureDescirptorParam(int gtex, int otex, float dwidth, float fwidth, float width, float height, float sigma);
	virtual void SetFeatureOrientationParam(int gtex, int width, int height, float sigma, int stex, float step);
	virtual void SetSimpleOrientationInput(int oTex, float sigma, float sigma_step);
	virtual void SetGenListEndParam(int ktex);
	virtual void SetGenListInitParam(int w, int h);
	virtual void SetMarginCopyParam(int xmax, int ymax);
};


#endif

