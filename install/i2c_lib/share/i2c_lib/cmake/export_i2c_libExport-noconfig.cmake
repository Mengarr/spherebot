#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "i2c_lib::i2c_interface" for configuration ""
set_property(TARGET i2c_lib::i2c_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(i2c_lib::i2c_interface PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libi2c_interface.a"
  )

list(APPEND _cmake_import_check_targets i2c_lib::i2c_interface )
list(APPEND _cmake_import_check_files_for_i2c_lib::i2c_interface "${_IMPORT_PREFIX}/lib/libi2c_interface.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
