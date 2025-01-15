# CMake package configuration file for the SofaUserInteraction module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaUserInteractionConfig.cmake.in                            ########

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

get_property(SofaUserInteraction_SENT_DEPRECATION_MESSAGE GLOBAL PROPERTY PROPERTY_SofaUserInteraction_SENT_DEPRECATION_MESSAGE SET)
if(NOT SofaUserInteraction_SENT_DEPRECATION_MESSAGE)
    message(WARNING "SofaUserInteraction module is being deprecated. It will be removed at v23.06. You may use Sofa.Component.Collision.Geometry, Sofa.Component.Collision.Detection.Algorithm, Sofa.Component.Collision.Detection.Intersection, Sofa.Component.Collision.Response.Contact  and Sofa.Component.Controller instead.")
endif()
set_property(GLOBAL PROPERTY PROPERTY_SofaUserInteraction_SENT_DEPRECATION_MESSAGE TRUE)

find_package(SofaMeshCollision QUIET REQUIRED)
find_package(SofaDeformable QUIET REQUIRED)
find_package(SofaGeneralMeshCollision QUIET REQUIRED)
find_package(SofaGeneralVisual QUIET REQUIRED)
find_package(SofaTopologyMapping QUIET REQUIRED)
find_package(SofaBoundaryCondition QUIET REQUIRED)
find_package(SofaGraphComponent QUIET REQUIRED)
find_package(Sofa.Component.Collision.Geometry QUIET REQUIRED)
find_package(Sofa.Component.Collision.Detection.Algorithm QUIET REQUIRED)
find_package(Sofa.Component.Collision.Detection.Intersection QUIET REQUIRED)
find_package(Sofa.Component.Collision.Response.Contact QUIET REQUIRED)
find_package(Sofa.Component.Controller QUIET REQUIRED)

if(NOT TARGET SofaUserInteraction)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaUserInteractionTargets.cmake")
endif()

check_required_components(SofaUserInteraction)
