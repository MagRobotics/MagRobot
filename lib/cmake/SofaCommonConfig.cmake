# CMake package configuration file for SofaCommon
### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaCommonConfig.cmake.in                            ########

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

set(SOFACOMMON_MODULES SofaSimpleFem;SofaRigid;SofaDeformable;SofaObjectInteraction;SofaMeshCollision;SofaEngine;SofaExplicitOdeSolver;SofaImplicitOdeSolver;SofaLoader)

message(DEPRECATION
    " SofaCommon is deprecated.\n"
    " You must now explicitely find (find_package) and link (target_link_library) the modules you need within: \n"
    " ${SOFACOMMON_MODULES}"
)

foreach(module ${SOFACOMMON_MODULES})
    find_package(${module} QUIET REQUIRED)
endforeach()

if(NOT TARGET SofaCommon)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaCommonTargets.cmake")
endif()
check_required_components(SofaCommon)
