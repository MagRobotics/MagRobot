# CMake package configuration file for the SofaGeneralEngine module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../bin")
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../lib")
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaGeneralEngineConfig.cmake.in                            ########

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

find_package(Sofa.GL.Component.Engine QUIET)

get_property(SofaGeneralEngine_SENT_DEPRECATION_MESSAGE GLOBAL PROPERTY PROPERTY_SofaGeneralEngine_SENT_DEPRECATION_MESSAGE SET)
if(NOT SofaGeneralEngine_SENT_DEPRECATION_MESSAGE)
    message(WARNING "${PROJECT_NAME} module is deprecated. It will be removed at v23.06. Use Sofa.Component.Engine.Analyze, Sofa.Component.Engine.Generate, Sofa.Component.Engine.Select and Sofa.Component.Engine.Transform instead.")
    if(Sofa.GL.Component.Engine_FOUND)
        message(WARNING "Moreover, use Sofa.GL.Component.Engine if you need TextureInterpolation.")
    endif()
endif()
set_property(GLOBAL PROPERTY PROPERTY_SofaGeneralEngine_SENT_DEPRECATION_MESSAGE TRUE)

sofa_find_package(Sofa.Component.Engine.Analyze REQUIRED QUIET)
sofa_find_package(Sofa.Component.Engine.Generate REQUIRED QUIET)
sofa_find_package(Sofa.Component.Engine.Select REQUIRED QUIET)
sofa_find_package(Sofa.Component.Engine.Transform REQUIRED QUIET)

if(NOT TARGET SofaGeneralEngine)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaGeneralEngineTargets.cmake")
endif()

check_required_components(SofaGeneralEngine)
