# CMake package configuration file for the BeamAdapter plugin


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was BeamAdapterConfig.cmake.in                            ########

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

set(BEAMADAPTER_HAVE_SOFAIMPLICITFIELD 0)
set(BEAMADAPTER_HAVE_SOFADVANCEDCONSTRAINT )

find_package(Sofa.Simulation.Core QUIET REQUIRED)
find_package(Sofa.GL QUIET REQUIRED)
find_package(Sofa.Component.StateContainer QUIET REQUIRED)
find_package(Sofa.Component.Controller QUIET REQUIRED)
find_package(Sofa.Component.Topology.Container.Dynamic QUIET REQUIRED)
find_package(Sofa.Component.Topology.Mapping QUIET REQUIRED)
find_package(Sofa.Component.Collision.Geometry QUIET  REQUIRED)
find_package(Sofa.Component.Constraint.Projective QUIET REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian QUIET REQUIRED)

if(BEAMADAPTER_HAVE_SOFAIMPLICITFIELD)
    find_package(SofaImplicitField QUIET REQUIRED)
endif()

if(BEAMADAPTER_HAVE_SOFADVANCEDCONSTRAINT)
    find_package(SofaAdvancedConstraint QUIET REQUIRED)
endif()

if(NOT TARGET BeamAdapter)
    include("${CMAKE_CURRENT_LIST_DIR}/BeamAdapterTargets.cmake")
endif()

check_required_components(BeamAdapter)
