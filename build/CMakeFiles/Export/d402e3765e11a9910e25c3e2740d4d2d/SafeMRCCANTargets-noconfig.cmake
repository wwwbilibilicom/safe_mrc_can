#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SafeMRCCAN::safemrc_can" for configuration ""
set_property(TARGET SafeMRCCAN::safemrc_can APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(SafeMRCCAN::safemrc_can PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsafemrc_can.a"
  )

list(APPEND _cmake_import_check_targets SafeMRCCAN::safemrc_can )
list(APPEND _cmake_import_check_files_for_SafeMRCCAN::safemrc_can "${_IMPORT_PREFIX}/lib/libsafemrc_can.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
