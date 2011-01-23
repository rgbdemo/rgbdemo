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

ADD_EXECUTABLE(rgbd-viewer ${moc_sources} ${uic_sources}
               viewer.cpp
               GuiController.cpp GuiController.h
               ObjectDetector.cpp ObjectDetector.h
               View3dWindow.cpp View3dWindow.h
               FiltersWindow.cpp FiltersWindow.h
               DetectorWindow.cpp DetectorWindow.h
               RawImagesWindow.cpp RawImagesWindow.h
)

TARGET_LINK_LIBRARIES(rgbd-viewer
                      nestk)
