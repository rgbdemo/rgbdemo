#
# INSTALL_NOBASE_HEADER_FILES(prefix file file file ...)
# Will create install rules for those files of the list
# which are headers (.h or .txx).
# If .in files are given, the .in extension is removed.
#

MACRO(NESTK_INSTALL_NOBASE_HEADER_FILES prefix)
FOREACH(file ${ARGN})
  IF(${file} MATCHES "\\.(h|txx|hxx|hpp|ipp)(\\.in)?$")
    STRING(REGEX REPLACE "\\.in$" "" install_file ${file})
    GET_FILENAME_COMPONENT(dir ${install_file} PATH)
    STRING(REGEX REPLACE "^${CMAKE_INSTALL_PREFIX}/$" "" install_file ${file})
    #INSTALL_FILES(${prefix}/${dir} FILES ${install_file}) ### deprecated
    INSTALL(FILES ${install_file} DESTINATION ${prefix}/${dir})
  ENDIF(${file} MATCHES "\\.(h|txx|hxx|hpp|ipp)(\\.in)?$")
ENDFOREACH(file ${filelist})
ENDMACRO(NESTK_INSTALL_NOBASE_HEADER_FILES)

MACRO(QT4_WRAP_CPP_INLINE targetname )
# get include dirs
GET_DIRECTORY_PROPERTY(moc_includes_tmp INCLUDE_DIRECTORIES)
SET(moc_includes)
FOREACH(it ${moc_includes_tmp})
  SET(moc_includes ${moc_includes} "-I${it}")
ENDFOREACH(it)

FOREACH(it ${ARGN})
  GET_FILENAME_COMPONENT(it ${it} ABSOLUTE)
  GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)

  #SET(infile ${CMAKE_CURRENT_SOURCE_DIR}/${it})
  SET(outfile ${CMAKE_CURRENT_BINARY_DIR}/${outfile}.moc)
  ADD_CUSTOM_TARGET(${targetname}
                    ${QT_MOC_EXECUTABLE} ${moc_includes} -i -o ${outfile} ${it}
                   )
ENDFOREACH(it)
ENDMACRO(QT4_WRAP_CPP_INLINE)
