SET(NESTK_DEPS_INSTALL_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/share/nestk-deps/cmake")
SET(NESTK_DEPS_INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
SET(NESTK_DEPS_INSTALL_LIBRARY_DIR "${CMAKE_INSTALL_PREFIX}/lib")

SET(NESTK_DEPS_INSTALL_USE_FILE "${CMAKE_INSTALL_PREFIX}/share/nestk-deps/cmake/UseNestkDeps.cmake")

SET(NESTK_DEPS_LIBRARY_DEPENDS_FILE "NestkDepsLibraryDepends.cmake")
SET(NESTK_DEPS_INSTALL_LIBRARY_DEPENDS_FILE "${NESTK_DEPS_INSTALL_CMAKE_DIR}/NestkDepsLibraryDepends.cmake")
EXPORT_LIBRARY_DEPENDENCIES("${nestk_deps_BINARY_DIR}/cmake/${NESTK_DEPS_LIBRARY_DEPENDS_FILE}")

SET(NESTK_DEPS_USE_FILE  
    ${nestk_deps_BINARY_DIR}/cmake/UseNestkDeps.cmake
    CACHE FILEPATH "Nestk deps use file")
CONFIGURE_FILE(UseNestkDeps.cmake.in 
    "${NESTK_DEPS_USE_FILE}" @ONLY)
    
SET(NESTK_DEPS_CONFIG_FILE  
    ${nestk_deps_BINARY_DIR}/cmake/NestkDepsConfig.cmake
    CACHE FILEPATH "Nestk deps config file")
CONFIGURE_FILE(NestkDepsConfig.cmake.in 
    "${NESTK_DEPS_CONFIG_FILE}" @ONLY)

### INSTALL_FILES # deprecated
INSTALL(
  FILES
  ${NESTK_DEPS_CONFIG_FILE} #NestkDepsConfig.cmake
  ${NESTK_DEPS_USE_FILE}    #UseNestkDeps.cmake
  utils.cmake
  msvc.cmake
  #${NESTK_DEPS_LIBRARY_DEPENDS_FILE} # non existing file
  DESTINATION share/nestk-deps/cmake
)
