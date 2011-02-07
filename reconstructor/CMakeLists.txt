QT4_WRAP_CPP(moc_sources
  GuiController.h
  ModelAcquisitionController.h
  FiltersWindow.h
  RawImagesWindow.h
  ModelAcquisitionWindow.h
)

QT4_WRAP_UI(uic_sources
  RawImagesWindow.ui
  FiltersWindow.ui
  ModelAcquisitionWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE(rgbd-reconstructor MACOSX_BUNDLE ${moc_sources} ${uic_sources}
               reconstructor.cpp
               RawImagesWindow.h RawImagesWindow.cpp
               GuiController.cpp GuiController.h
               ModelAcquisitionController.cpp ModelAcquisitionController.h
               FiltersWindow.cpp FiltersWindow.h
               ModelAcquisitionWindow.cpp ModelAcquisitionWindow.h
)

TARGET_LINK_LIBRARIES(rgbd-reconstructor
                      nestk)
INSTALL(TARGETS rgbd-reconstructor DESTINATION bin)
SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
INSTALL_STANDALONE_BUNDLE(rgbd-reconstructor ${DIRS})
