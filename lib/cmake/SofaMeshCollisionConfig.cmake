# CMake package configuration file for the SofaMeshCollision module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaMeshCollisionConfig.cmake.in                            ########

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

get_property(SofaMeshCollision_SENT_DEPRECATION_MESSAGE GLOBAL PROPERTY PROPERTY_SofaMeshCollision_SENT_DEPRECATION_MESSAGE SET)
if(NOT SofaMeshCollision_SENT_DEPRECATION_MESSAGE)
    message(WARNING "SofaMeshCollision module is being deprecated. It will be removed at v23.06. You may use Sofa.Component.Collision.Geometry, Sofa.Component.Collision.Detection.Intersection, Sofa.Component.Collision.Response.Mapper and Sofa.Component.Collision.Response.Contact instead.")
endif()
set_property(GLOBAL PROPERTY PROPERTY_SofaMeshCollision_SENT_DEPRECATION_MESSAGE TRUE)

find_package(SofaBaseMechanics QUIET REQUIRED)
find_package(SofaBaseCollision QUIET REQUIRED)
find_package(SofaBaseTopology QUIET REQUIRED)
find_package(SofaObjectInteraction QUIET REQUIRED)
find_package(SofaRigid QUIET REQUIRED)
find_package(Sofa.Component.Collision.Geometry QUIET REQUIRED)
find_package(Sofa.Component.Collision.Detection.Intersection QUIET REQUIRED)
find_package(Sofa.Component.Collision.Response.Mapper QUIET REQUIRED)
find_package(Sofa.Component.Collision.Response.Contact QUIET REQUIRED)

if(NOT TARGET SofaMeshCollision)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaMeshCollisionTargets.cmake")
endif()

check_required_components(SofaMeshCollision)
