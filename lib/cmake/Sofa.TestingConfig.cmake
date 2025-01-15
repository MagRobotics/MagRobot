# CMake package configuration file for the Sofa.Testing module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Sofa.TestingConfig.cmake.in                            ########

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

find_package(Threads QUIET REQUIRED)
find_package(GTest CONFIG QUIET REQUIRED)
find_package(SofaGTestMain QUIET REQUIRED)
find_package(Sofa.Helper QUIET REQUIRED)
find_package(Sofa.DefaultType QUIET REQUIRED)
find_package(Sofa.Core QUIET REQUIRED)
find_package(Sofa.Simulation.Graph QUIET REQUIRED)

if(NOT TARGET Sofa.Testing)
    include("${CMAKE_CURRENT_LIST_DIR}/Sofa.TestingTargets.cmake")
endif()

check_required_components(Sofa.Testing)
