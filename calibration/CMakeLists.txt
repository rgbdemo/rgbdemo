SET(common_sources calibration_common.h calibration_common.cpp)

ADD_EXECUTABLE(annotate_image annotate_image.cpp ${common_sources})
TARGET_LINK_LIBRARIES(annotate_image nestk)

ADD_EXECUTABLE(calibrate_kinect calibrate_kinect.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate_kinect nestk)

NESTK_ADD_EXECUTABLE(calibrate_projector calibrate_projector.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate_projector nestk)

NESTK_ADD_EXECUTABLE(calibrate-multiple-kinects calibrate_multiple_kinects.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-multiple-kinects nestk)

NESTK_ADD_EXECUTABLE(calibrate-openni-intrinsics calibrate_openni_intrinsics.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-openni-intrinsics nestk)

NESTK_ADD_EXECUTABLE(calibrate-openni-depth calibrate_openni_depth.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate-openni-depth nestk)

NESTK_ADD_EXECUTABLE(calibrate_kinect_ir calibrate_kinect_ir.cpp ${common_sources})
TARGET_LINK_LIBRARIES(calibrate_kinect_ir nestk)
