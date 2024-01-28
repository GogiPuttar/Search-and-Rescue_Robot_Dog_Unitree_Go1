#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "unitree_kinematics::frame_main" for configuration ""
set_property(TARGET unitree_kinematics::frame_main APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(unitree_kinematics::frame_main PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/frame_main"
  )

list(APPEND _IMPORT_CHECK_TARGETS unitree_kinematics::frame_main )
list(APPEND _IMPORT_CHECK_FILES_FOR_unitree_kinematics::frame_main "${_IMPORT_PREFIX}/bin/frame_main" )

# Import target "unitree_kinematics::unitree_kinematics" for configuration ""
set_property(TARGET unitree_kinematics::unitree_kinematics APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(unitree_kinematics::unitree_kinematics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libunitree_kinematics.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS unitree_kinematics::unitree_kinematics )
list(APPEND _IMPORT_CHECK_FILES_FOR_unitree_kinematics::unitree_kinematics "${_IMPORT_PREFIX}/lib/libunitree_kinematics.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
