QT4_WRAP_CPP(moc_sources
  GuiController.h
  DetectorWindow.h
  View3dWindow.h
  FiltersWindow.h
  RawImagesWindow.h
)

QT4_WRAP_UI(uic_sources
  RawImagesWindow.ui
  View3dWindow.ui
  FiltersWindow.ui
  DetectorWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

SET(rgbd_viewer_sources ${moc_sources} ${uic_sources}
               viewer.cpp
               GuiController.cpp GuiController.h
               ObjectDetector.cpp ObjectDetector.h
               View3dWindow.cpp View3dWindow.h
               FiltersWindow.cpp FiltersWindow.h
               DetectorWindow.cpp DetectorWindow.h
               RawImagesWindow.cpp RawImagesWindow.h
)

IF (BUILD_MACOSX_BUNDLE)
   ADD_EXECUTABLE(rgbd-viewer MACOSX_BUNDLE ${rgbd_viewer_sources}) 
   SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
   INSTALL_STANDALONE_BUNDLE(rgbd-viewer ${DIRS})
ELSE()
   ADD_EXECUTABLE(rgbd-viewer ${rgbd_viewer_sources})
ENDIF()

TARGET_LINK_LIBRARIES(rgbd-viewer
                      nestk)

INSTALL(TARGETS rgbd-viewer DESTINATION bin)

