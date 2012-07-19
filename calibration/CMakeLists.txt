SET(common_sources calibration_common.h calibration_common.cpp)

IF (BUILD_OBSOLETE)
  ADD_EXECUTABLE(annotate-image annotate_image.cpp ${common_sources})
  TARGET_LINK_LIBRARIES(annotate_image nestk)

  ADD_EXECUTABLE(calibrate-kinect calibrate_kinect.cpp ${common_sources})
  TARGET_LINK_LIBRARIES(calibrate_kinect nestk)
ENDIF()

NESTK_ADD_EXECUTABLE(calibrate-projector calibrate_projector.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-projector nestk)

NESTK_ADD_EXECUTABLE(calibrate-multiple-kinects calibrate_multiple_kinects.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-multiple-kinects nestk)

NESTK_ADD_EXECUTABLE(calibrate-openni-intrinsics calibrate_openni_intrinsics.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-openni-intrinsics nestk)

NESTK_ADD_EXECUTABLE(calibrate-openni-depth calibrate_openni_depth.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-openni-depth nestk)

NESTK_ADD_EXECUTABLE(calibrate-kinect-ir calibrate_kinect_ir.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-kinect-ir nestk)

IF (NESTK_USE_PMDSDK)
    NESTK_ADD_EXECUTABLE(calibrate-pmdnano calibrate_pmdnano.cpp ${common_sources})
    TARGET_LINK_LIBRARIES(calibrate-pmdnano nestk)
ENDIF()
