#
# Root nestk
#
#   nicolas.burrus@ensta.fr

CMAKE_MINIMUM_REQUIRED(VERSION 2.2)

PROJECT(nestk_deps)
SET(NESTK_DEPS_VERSION 0.1)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${nestk_deps_SOURCE_DIR}/cmake CACHE STATIC "Nestk deps module path")

INCLUDE( ${nestk_deps_SOURCE_DIR}/cmake/utils.cmake )
INCLUDE( ${nestk_deps_SOURCE_DIR}/cmake/msvc.cmake )

################################################################################
## OpenCV
IF (IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/opencv")
  OPTION(USE_EXTERNAL_OPENCV "Use external OpenCV" 0)
ELSE ()
  OPTION(USE_EXTERNAL_OPENCV "Use external OpenCV" 1)
ENDIF()
IF (NOT USE_EXTERNAL_OPENCV)
  ADD_SUBDIRECTORY ( opencv )
ENDIF(NOT USE_EXTERNAL_OPENCV)

## GLEW
set(GLEW_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/glew/include" CACHE FILEPATH "" FORCE)
set(GLEW_LIBRARIES "glew" CACHE FILEPATH "" FORCE)
ADD_SUBDIRECTORY("${nestk_deps_SOURCE_DIR}/glew")

################################################################################
# libfreeinect
################################################################################
OPTION(USE_EXTERNAL_FREENECT "Use external Freenect" 0)
IF(NOT USE_EXTERNAL_FREENECT AND USE_FREENECT)
    ## PTHREADS
    if(WIN32)
      IF (NOT THREADS_PTHREADS_INCLUDE_DIR)
        set(THREADS_PTHREADS_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/pthreads/include" CACHE FILEPATH "" FORCE)
        set(THREADS_USE_PTHREADS_WIN32 true)
        FIND_PACKAGE(Threads)
        set(PTHREAD_LIBRARY ${CMAKE_THREAD_LIBS_INIT} CACHE FILEPATH "" FORCE)
      ENDIF()
      include_directories("${nestk_deps_SOURCE_DIR}/libfreenect/platform/windows")
      include_directories("${nestk_deps_SOURCE_DIR}/libfreenect/platform/windows/libusb10emu")
    else()
      set(PTHREAD_LIBRARY pthread)
    endif()

    ## LIBUSB
    if(WIN32)
      IF (NOT LIBUSB_1_INCLUDE_DIR)
        set(LIBUSB_1_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/libusb-1.0/include" CACHE FILEPATH "" FORCE)
		IF (CMAKE_COMPILER_IS_GNUCXX)
          set(LIBUSB_1_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/libusb-1.0/lib/gcc/libusb.a" CACHE FILEPATH "" FORCE)
		ELSE()
		  set(LIBUSB_1_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/libusb-1.0/lib/msvc/libusb.lib" CACHE FILEPATH "" FORCE)
		ENDIF()
      ENDIF()
    endif()

    ## GLUT
    if(WIN32)
      IF (NOT GLUT_INCLUDE_DIR)
	    IF (CMAKE_COMPILER_IS_GNUCXX)
          set(GLUT_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/freeglut/include" CACHE FILEPATH "" FORCE)
          set(GLUT_glut_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/freeglut/lib/libfreeglut.a" CACHE FILEPATH "" FORCE)
		ELSE()
		  set(GLUT_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/glut-msvc" CACHE FILEPATH "" FORCE)
          set(GLUT_glut_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/glut-msvc/glut32.lib" CACHE FILEPATH "" FORCE)
		ENDIF()
      ENDIF()
    endif()
    ADD_SUBDIRECTORY(libfreenect)
ENDIF()

################################################################################
# eigen
################################################################################
ADD_SUBDIRECTORY(eigen)

################################################################################
# cmake scripts
################################################################################
ADD_SUBDIRECTORY(cmake)
