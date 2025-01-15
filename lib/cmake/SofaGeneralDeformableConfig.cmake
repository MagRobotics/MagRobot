# CMake package configuration file for the SofaGeneralDeformable module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../bin")
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../lib")
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaGeneralDeformableConfig.cmake.in                            ########

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

get_property(SofaGeneralDeformable_SENT_DEPRECATION_MESSAGE GLOBAL PROPERTY PROPERTY_SofaGeneralDeformable_SENT_DEPRECATION_MESSAGE SET)
if(NOT SofaGeneralDeformable_SENT_DEPRECATION_MESSAGE)
    message(WARNING "SofaGeneralDeformable module is deprecated. It will be removed at v23.06. Use Sofa.Component.SolidMechanics.Spring and Sofa.Component.SolidMechanics.TensorMass instead.")
endif()
set_property(GLOBAL PROPERTY PROPERTY_SofaGeneralDeformable_SENT_DEPRECATION_MESSAGE TRUE)

find_package(Sofa.Component.SolidMechanics.Spring QUIET REQUIRED)
find_package(Sofa.Component.SolidMechanics.TensorMass QUIET REQUIRED)

if(NOT TARGET SofaGeneralDeformable)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaGeneralDeformableTargets.cmake")
endif()

check_required_components(SofaGeneralDeformable)
