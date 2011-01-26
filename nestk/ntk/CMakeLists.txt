#
# Root nestk
#
#   nicolas.burrus@ensta.fr

SET ( ntk_sources

      core.h
      ntk.h
    )

SET (ntk_sources ${ntk_sources}

     camera/calibration.h
     camera/calibration.cpp
     camera/file_grabber.h
     camera/file_grabber.cpp
     camera/opencv_grabber.h
     camera/opencv_grabber.cpp
     camera/rgbd_frame_recorder.h
     camera/rgbd_frame_recorder.cpp
     camera/rgbd_grabber.h
     camera/rgbd_grabber.cpp
     camera/rgbd_image.h
     camera/rgbd_image.cpp
     camera/rgbd_processor.h
     camera/rgbd_processor.cpp
)

IF (USE_FREENECT)
  SET (ntk_sources ${ntk_sources}
       camera/kinect_grabber.h
       camera/kinect_grabber.cpp)
ENDIF()


SET (ntk_sources ${ntk_sources}

     geometry/affine_transform.h
     geometry/affine_transform.cpp
     geometry/eigen_utils.h
     geometry/plane.h
     geometry/plane.cpp
     geometry/polygon.h
     geometry/polygon.cpp
     geometry/pose_3d.h
     geometry/pose_3d.cpp
     geometry/relative_pose_estimator.h
     geometry/relative_pose_estimator.cpp
     geometry/similarity_transform.h
     geometry/similarity_transform.cpp
)

IF (HAVE_OPENCL)
  SET (ntk_sources ${ntk_sources}
       gpu/opencl.cpp
       gpu/opencl.h
   )
ENDIF()

SET (ntk_sources ${ntk_sources}

     image/color_model.h
     image/color_model.cpp
     image/feature.h
     image/feature.cpp
     image/sift.h
     image/sift.cpp
)

IF (GLEW_INCLUDE_DIR)
SET (ntk_sources ${ntk_sources}
     image/sift_gpu.h
     image/sift_gpu.cpp
     image/siftgpu/SiftGPU.h
     image/siftgpu/SiftGPU.cpp
     image/siftgpu/FrameBufferObject.cpp
     image/siftgpu/FrameBufferObject.h
     image/siftgpu/GlobalUtil.cpp
     image/siftgpu/GlobalUtil.h
     image/siftgpu/GLTexImage.cpp
     image/siftgpu/GLTexImage.h
     image/siftgpu/LiteWindow.h
     image/siftgpu/ProgramGLSL.cpp
     image/siftgpu/ProgramGLSL.h
     image/siftgpu/ProgramGPU.cpp
     image/siftgpu/ProgramGPU.h
     image/siftgpu/PyramidGL.cpp
     image/siftgpu/PyramidGL.h
     image/siftgpu/ShaderMan.cpp
     image/siftgpu/ShaderMan.h
     image/siftgpu/SiftPyramid.cpp
     image/siftgpu/SiftPyramid.h
)
ENDIF()

SET (ntk_sources ${ntk_sources}

     mesh/ply.h
     mesh/ply.cpp
     mesh/mesh.h
     mesh/mesh.cpp
     mesh/mesh_generator.h
     mesh/mesh_generator.cpp
     mesh/mesh_renderer.h
     mesh/mesh_renderer.cpp
     mesh/rgbd_modeler.h
     mesh/rgbd_modeler.cpp
     mesh/surfels_rgbd_modeler.h
     mesh/surfels_rgbd_modeler.cpp
    )

SET (ntk_sources ${ntk_sources}

     numeric/cost_function.h
     numeric/cost_function.cpp
     numeric/levenberg_marquart_minimizer.h
     numeric/levenberg_marquart_minimizer.cpp
     numeric/cost_function.h
     numeric/utils.h
     numeric/utils.cpp
     numeric/differential_evolution_solver.h
     numeric/differential_evolution_solver.cpp
     numeric/differential_evolution_minimizer.h
     numeric/differential_evolution_minimizer.cpp
    )

SET (ntk_sources ${ntk_sources}

     stats/distributions.h
     stats/distributions.cpp
     stats/estimation.cpp
     stats/estimation.h
     stats/histogram.h
     stats/histogram.cpp
     stats/hypothesis_testing.cpp
     stats/hypothesis_testing.h
     stats/hypothesis_testing.hxx
     stats/moments.h
     stats/moments.cpp
     stats/moments.hxx
    )


SET (ntk_sources ${ntk_sources}

     thread/event.h
     thread/event.cpp
     thread/utils.h
     thread/utils.cpp
)

SET (ntk_sources ${ntk_sources}

     utils/arg.h
     utils/arg.cpp
     utils/common.h
     utils/debug.h
     utils/debug.cpp
     utils/opencv_utils.h
     utils/opencv_utils.cpp
     utils/qt_utils.h
     utils/qt_utils.cpp
     utils/stl.h
     utils/serializable.h
     utils/time.h
     utils/xml_parser.h
     utils/xml_parser.cpp
     utils/xml_serializable.h
     utils/xml_serializable.cpp
    )

  SET (ntk_sources ${ntk_sources}

    gui/image_widget.h
    gui/image_widget.cpp
    gui/screen_grabber.h
    gui/screen_grabber.cpp

    mesh/mesh_viewer.h
    mesh/mesh_viewer.cpp
  )

  QT4_WRAP_CPP(moc_sources
    gui/image_widget.h
    mesh/mesh_viewer.h
    thread/event.h
  )

IF (WIN32 OR APPLE)
  ADD_LIBRARY(nestk STATIC ${ntk_sources} ${moc_sources})
ELSE (WIN32 OR APPLE)
  ADD_LIBRARY(nestk STATIC ${ntk_sources} ${moc_sources})
ENDIF (WIN32 OR APPLE)

IF (WIN32)
  set(WINMM_LIBRARY winmm)
ENDIF()

TARGET_LINK_LIBRARIES(nestk
  ${QT_LIBRARIES}
  ${GSL_LIBRARIES}
  ${OPENCL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLUT_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBS}
  ${X11_LIBRARY}
  ${LIBUSB_1_LIBRARY}
  ${PMD_LIBRARIES}
  ${FREENECT_LIBRARY}
  ${WINMM_LIBRARY}
  ${PTHREADS_LIBRARY})

SET_TARGET_PROPERTIES(nestk PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
  RUNTIME_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
)

INSTALL(TARGETS nestk DESTINATION lib)
NESTK_INSTALL_NOBASE_HEADER_FILES(include/ntk ${ntk_sources})
