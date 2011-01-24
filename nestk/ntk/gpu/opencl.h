// Wrapper extracted from http://enja.org/, courtesy of Ian Johnson

#ifndef NTK_GPU_OPENCL_H
#define NTK_GPU_OPENCL_H

#define __CL_ENABLE_EXCEPTIONS

#include <ntk/core.h>
#include <CL/cl.hpp>

namespace ntk
{

//NVIDIA helper functions    
const char* oclErrorString(cl_int error);
cl_int oclGetPlatformID(cl_platform_id* clSelectedPlatformID);

class CL
{
public:
    CL();
/*
    std::vector<Buffer> buffers;
    std::vector<Program> programs;
    std::vector<Kernel> kernels;

    int addBuffer(Buffer buff);
    int addProgram(Program prog);
    int addKernel(Kernel kern);
*/

    cl::Context context;
    cl::CommandQueue queue;

    std::vector<cl::Device> devices;
    int deviceUsed;

    //error checking stuff
    int err;
    cl::Event event;

    //setup an OpenCL context that shares with OpenGL
    void setup_gl_cl();

    cl::Program loadProgram(std::string path);
    cl::Program loadProgramFromString(const std::string& source_code);
    cl::Kernel loadKernel(std::string path, std::string name);
    cl::Kernel loadKernelFromString(const std::string& source_code, const std::string& name);

    //TODO add oclErrorString to the class
    //move from util.h/cpp
};

template <class T>
class Buffer
{
public:
    Buffer(){ cli=NULL; vbo_id=0; };
    //create an OpenCL buffer from existing data
    Buffer(CL *cli, const std::vector<T> &data);
    Buffer(CL *cli, const std::vector<T> &data, unsigned int memtype);
    //create a OpenCL BufferGL from a vbo_id
    //if managed is true then the destructor will delete the VBO
    Buffer(CL *cli, GLuint vbo_id);
    ~Buffer();

  cl_mem getDevicePtr() { return cl_buffer[0](); }

    //need to acquire and release arrays from OpenGL context if we have a VBO
    void acquire();
    void release();

    void copyToDevice(const std::vector<T> &data);
    void copyRawToDevice(const T* data, int num); //copy
    //pastes the data over the current array starting at [start]
    void copyToDevice(const std::vector<T> &data, int start);
    std::vector<T> copyToHost(int num);

    void set(T val);
    void set(const std::vector<T> &data);

private:
     //we will want to access buffers by name when going across systems
    //std::string name;
    //the actual buffer handled by the Khronos OpenCL c++ header
    //cl::Memory cl_buffer;
    std::vector<cl::Memory> cl_buffer;

    CL *cli;

    //if this is a VBO we store its id
    GLuint vbo_id;
};

class Kernel
{
public:
    Kernel(){cli = NULL;};
    Kernel(CL *cli, std::string name, std::string source);

    //we will want to access buffers by name when going accross systems
    std::string name;
    std::string source;

    CL *cli;
    //we need to build a program to have a kernel
    cl::Program program;

    //the actual OpenCL kernel object
    cl::Kernel kernel;

    template <class T> void setArg(int arg, T val);
    void setArgShared(int arg, int nb_bytes);


    //assumes null range for worksize offset and local worksize
    void execute(int ndrange);
    //later we will make more execute routines to give more options
    void execute(int ndrange, int workgroup_size);

};

template <class T> void Kernel::setArg(int arg, T val)
{
    try
    {
        kernel.setArg(arg, val);
    }
    catch (cl::Error er) {
        printf("ERROR: %s(%s)\n", er.what(), oclErrorString(er.err()));
    }

}

template <class T>
Buffer<T>::Buffer(CL *cli, const std::vector<T> &data)
{
    this->cli = cli;
    //this->data = data;

    cl_buffer.push_back(cl::Buffer(cli->context, CL_MEM_READ_WRITE, data.size()*sizeof(T), NULL, &cli->err));
    copyToDevice(data);


}

template <class T>
Buffer<T>::Buffer(CL *cli, const std::vector<T> &data, unsigned int memtype)
{
    this->cli = cli;
    //this->data = data;

    cl_buffer.push_back(cl::Buffer(cli->context, memtype, data.size()*sizeof(T), NULL, &cli->err));
    copyToDevice(data);


}


template <class T>
Buffer<T>::Buffer(CL *cli, GLuint vbo_id)
{
    this->cli = cli;
    cl_buffer.push_back(cl::BufferGL(cli->context, CL_MEM_READ_WRITE, vbo_id, &cli->err));
}

template <class T>
Buffer<T>::~Buffer()
{
}

template <class T>
void Buffer<T>::acquire()
{
    cli->err = cli->queue.enqueueAcquireGLObjects(&cl_buffer, NULL, &cli->event);
    cli->queue.finish();
}


template <class T>
void Buffer<T>::release()
{
    cli->err = cli->queue.enqueueReleaseGLObjects(&cl_buffer, NULL, &cli->event);
    cli->queue.finish();
}


template <class T>
void Buffer<T>::copyToDevice(const std::vector<T> &data)
{
    //TODO clean up this memory/buffer issue (nasty pointer casting)
    cli->err = cli->queue.enqueueWriteBuffer(*((cl::Buffer*)&cl_buffer[0]), CL_TRUE, 0, data.size()*sizeof(T), &data[0], NULL, &cli->event);
    cli->queue.finish();

}

template <class T>
void Buffer<T>::copyToDevice(const std::vector<T> &data, int start)
{
    //TODO clean up this memory/buffer issue (nasty pointer casting)
    cli->err = cli->queue.enqueueWriteBuffer(*((cl::Buffer*)&cl_buffer[0]), CL_TRUE, start*sizeof(T), data.size()*sizeof(T), &data[0], NULL, &cli->event);
    cli->queue.finish();

}

template <class T>
void Buffer<T>::copyRawToDevice(const T* data, int num)
{
    //TODO clean up this memory/buffer issue (nasty pointer casting)
    cli->err = cli->queue.enqueueWriteBuffer(*((cl::Buffer*)&cl_buffer[0]), CL_TRUE, 0, num*sizeof(T), data, NULL, &cli->event);
    cli->queue.finish();

}

template <class T>
std::vector<T> Buffer<T>::copyToHost(int num)
{
    //TODO clean up this memory/buffer issue
    std::vector<T> data(num);
    //TODO pass back a pointer instead of a copy
    //std::vector<T> data = new std::vector<T>(num);
    cli->err = cli->queue.enqueueReadBuffer(*((cl::Buffer*)&cl_buffer[0]), CL_TRUE, 0, data.size()*sizeof(T), &data[0], NULL, &cli->event);
    cli->queue.finish();
    return data;

}

} // ntk

#endif // !NTK_GPU_OPENCL_H
