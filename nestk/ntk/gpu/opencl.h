// Wrapper extracted from http://enja.org/, courtesy of Ian Johnson

#ifndef NTK_GPU_OPENCL_H
#define NTK_GPU_OPENCL_H

#define __CL_ENABLE_EXCEPTIONS

#include <ntk/core.h>
#include <ntk/utils/debug.h>

#include <CL/cl.hpp>

#define opencl_stringify(Code) #Code

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

class BufferBase
{
public:
  enum MemorySharing { NoMemSharing = 0, UseHostMemType = 1, UsePinnedMemory = 2 };
  enum MemAccessType { ReadWrite = CL_MEM_READ_WRITE,
                       ReadOnly = CL_MEM_READ_ONLY,
                       WriteOnly = CL_MEM_WRITE_ONLY };
};

template <class T>
class Buffer : public BufferBase
{
public:
  Buffer(){ cli=NULL; vbo_id=0; };

  /*!
   * Create an OpenCL buffer from existing data.
   * Data must be provided if UseHostMemType or UsePinnedMemory is set.
   * Takes a T** since UsePinnedMemory will return a new pointer.
   */
  Buffer(CL *cli,
         unsigned int num_elements,
         MemAccessType = ReadWrite,
         T** data = 0,
         MemorySharing flags = NoMemSharing);

  //create a OpenCL BufferGL from a vbo_id
  //if managed is true then the destructor will delete the VBO
  Buffer(CL *cli, GLuint vbo_id);
  ~Buffer();

  cl_mem getDevicePtr() { return cl_buffer[0](); }

  //need to acquire and release arrays from OpenGL context if we have a VBO
  void acquireGL();
  void releaseGL();

  /*! Copy data from CPU to GPU. */
  void copyToDevice(const T* data, unsigned num_elements);

  /*! Copy data from GPU to CPU. */
  void copyToHost(T* data, unsigned num_elements);

  /*! Get a mapped pointer for pinned memory. */
  void mapToHost(T** mapped_ptr, unsigned num_elements);

  /*! Release a mapped pointer. */
  void unmapToHost(T* mapped_ptr);

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
  Kernel(CL *cli, const std::string& name, const std::string& source);

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
Buffer<T>::Buffer(CL *cli,
                  unsigned int num_elements,
                  MemAccessType access,
                  T** data,
                  MemorySharing sharing)
{
  this->cli = cli;

  switch (sharing)
  {
  case NoMemSharing:
    cl_buffer.push_back(cl::Buffer(cli->context,
                                   (int) access,
                                   num_elements*sizeof(T),
                                   0, &cli->err));
    break;
  case UseHostMemType:
    ntk_assert(data && *data, "You must provide data when using HostMemType");
    cl_buffer.push_back(cl::Buffer(cli->context,
                                   (int) access | CL_MEM_USE_HOST_PTR,
                                   num_elements*sizeof(T),
                                   *data,
                                   &cli->err));
    break;
  case UsePinnedMemory:
    ntk_assert(data, "You must provide data when using PinnedMemory");
    cl_buffer.push_back(cl::Buffer(cli->context,
                                   (int) access | CL_MEM_ALLOC_HOST_PTR,
                                   num_elements*sizeof(T),
                                   0,
                                   &cli->err));
    *data = (T*) cli->queue.enqueueMapBuffer(*((cl::Buffer*)&cl_buffer[0]),
                                             CL_TRUE, CL_MAP_READ,
                                             0, num_elements*sizeof(T),
                                             0, &cli->event, &cli->err);
    break;
  };
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
void Buffer<T>::acquireGL()
{
  cli->err = cli->queue.enqueueAcquireGLObjects(&cl_buffer, NULL, &cli->event);
  cli->queue.finish();
}


template <class T>
void Buffer<T>::releaseGL()
{
  cli->err = cli->queue.enqueueReleaseGLObjects(&cl_buffer, NULL, &cli->event);
}

template <class T>
void Buffer<T>::copyToDevice(const T* data, unsigned num_elements)
{
  cli->err = cli->queue.enqueueWriteBuffer(*((cl::Buffer*)&cl_buffer[0]),
                                           CL_TRUE, 0,
                                           num_elements*sizeof(T),
                                           data, NULL, &cli->event);
}

template <class T>
void Buffer<T>::copyToHost(T* data, unsigned num_elements)
{
  cli->err = cli->queue.enqueueReadBuffer(*((cl::Buffer*)&cl_buffer[0]),
                                          CL_TRUE, /* blocking */
                                          0,
                                          num_elements*sizeof(T),
                                          data,
                                          NULL, &cli->event);
}

template <class T>
void Buffer<T>::mapToHost(T** mapped_ptr, unsigned num_vector_elements)
{
  //TODO pass back a pointer instead of a copy
  //std::vector<T> data = new std::vector<T>(num);
  *mapped_ptr = cli->queue.enqueueMapBuffer(*((cl::Buffer*)&cl_buffer[0]),
                                            CL_TRUE, CL_MAP_READ,
                                            0, num_vector_elements*sizeof(T),
                                            NULL, &cli->event, &cli->err);
}

template <class T>
void Buffer<T>::unmapToHost(T* mapped_ptr)
{
  cli->err = cli->queue.enqueueUnmapMemObject(*((cl::Buffer*)&cl_buffer[0]),
                                              mapped_ptr,
                                              NULL, &cli->event);
}

} // ntk

#endif // !NTK_GPU_OPENCL_H
