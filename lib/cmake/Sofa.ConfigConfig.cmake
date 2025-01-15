# CMake package configuration file for the Sofa.Config module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Sofa.ConfigConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(Sofa_VERSION 22.06.99)

# Setup CMAKE_MODULE_PATH, CMAKE_PREFIX_PATH, CMAKE_INCLUDE_PATH
string(REGEX REPLACE "/lib/cmake/?.*" "" SOFA_ROOT "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH  "${CMAKE_CURRENT_LIST_DIR}") # Needed to do: include(SofaMacros)
list(APPEND CMAKE_MODULE_PATH  "${SOFA_ROOT}/lib/cmake/Modules") # Needed to do: find_package(Eigen3)
list(APPEND CMAKE_INCLUDE_PATH "${SOFA_ROOT}/include/extlibs/WinDepPack") # Needed to do: #include <zlib.h>
list(APPEND CMAKE_PREFIX_PATH  "${SOFA_ROOT}/plugins") # Needed to do: find_package(CImgPlugin)

# Help RELOCATABLE plugins to resolve their dependencies.
# See SofaMacrosInstall.cmake for usage of this property.
define_property(TARGET
    PROPERTY "RELOCATABLE_INSTALL_DIR"
    BRIEF_DOCS "Install directory of RELOCATABLE target"
    FULL_DOCS "Install directory of RELOCATABLE target"
    )

include(CMakeDependentOption)
include(SofaMacros)

set(SOFA_WITH_OPENGL "ON")
if(SOFA_WITH_OPENGL)
    set(SOFA_NO_OPENGL FALSE) # Deprecated, use SOFA_WITH_OPENGL instead
else()
    set(SOFA_NO_OPENGL TRUE) # Deprecated, use SOFA_WITH_OPENGL instead
endif()

set(SOFA_WITH_DEVTOOLS "ON")
set(SOFA_WITH_THREADING "ON")
set(SOFA_WITH_DEPRECATED_COMPONENTS "ON")

if(NOT TARGET Sofa.Config)
    include("${CMAKE_CURRENT_LIST_DIR}/Sofa.ConfigTargets.cmake")
endif()

check_required_components(Sofa.Config)
