QT4_WRAP_CPP(moc_sources
  GuiController.h
  RawImagesWindow.h
)

QT4_WRAP_UI(uic_sources
  RawImagesWindow.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE(rgbd-skeletor MACOSX_BUNDLE ${moc_sources} ${uic_sources}
               skeletor.cpp
               RawImagesWindow.h RawImagesWindow.cpp
               GuiController.cpp GuiController.h
)

TARGET_LINK_LIBRARIES(rgbd-skeletor
                      nestk)
INSTALL(TARGETS rgbd-skeletor DESTINATION bin)
SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
INSTALL_STANDALONE_BUNDLE(rgbd-skeletor ${DIRS})
