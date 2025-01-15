# CMake package configuration file for Sofa.Component.SolidMechanics

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Sofa.Component.SolidMechanicsConfig.cmake.in                            ########

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

set(SOFACOMPONENTSOLIDMECHANICS_TARGETS Sofa.Component.SolidMechanics.Spring;Sofa.Component.SolidMechanics.FEM;Sofa.Component.SolidMechanics.TensorMass)

foreach(target ${SOFACOMPONENTSOLIDMECHANICS_TARGETS})
    find_package(${target} QUIET REQUIRED)
endforeach()

if(NOT TARGET Sofa.Component.SolidMechanics)
    include("${CMAKE_CURRENT_LIST_DIR}/Sofa.Component.SolidMechanicsTargets.cmake")
endif()
check_required_components(Sofa.Component.SolidMechanics)
