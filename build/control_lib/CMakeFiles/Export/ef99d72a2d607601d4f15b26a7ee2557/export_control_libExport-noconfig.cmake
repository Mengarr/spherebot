#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "control_lib::control_interfaces" for configuration ""
set_property(TARGET control_lib::control_interfaces APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(control_lib::control_interfaces PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcontrol_interfaces.a"
  )

list(APPEND _cmake_import_check_targets control_lib::control_interfaces )
list(APPEND _cmake_import_check_files_for_control_lib::control_interfaces "${_IMPORT_PREFIX}/lib/libcontrol_interfaces.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
