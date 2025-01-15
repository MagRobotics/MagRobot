# CMake package configuration file for SofaGui

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaGuiConfig.cmake.in                            ########

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

set(SOFAGUI_TARGETS SofaGuiCommon;SofaGuiQt)

set(SOFAGUI_HAVE_SOFAHEADLESSRECORDER 0)
set(SOFAGUI_HAVE_SOFAGUIQT 1)

find_package(SofaGuiCommon QUIET REQUIRED)

if(SOFAGUI_HAVE_SOFAGUIQT)
    find_package(SofaGuiQt QUIET REQUIRED)
endif()

if(SOFAGUI_HAVE_SOFAHEADLESSRECORDER)
    find_package(SofaHeadlessRecorder QUIET REQUIRED)
endif()

if(NOT TARGET SofaGui)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaGuiTargets.cmake")
endif()

check_required_components(SofaGui)
