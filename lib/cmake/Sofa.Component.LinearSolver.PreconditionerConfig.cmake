# CMake package configuration file for the Sofa.Component.LinearSolver.Preconditioner module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Sofa.Component.LinearSolver.PreconditionerConfig.cmake.in                            ########

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

find_package(Sofa.Simulation.Core QUIET REQUIRED)
find_package(Sofa.Component.ODESolver.Backward QUIET REQUIRED)
find_package(Sofa.Component.LinearSolver.Iterative QUIET REQUIRED)
find_package(Sofa.Component.LinearSolver.Direct QUIET REQUIRED)

if(NOT TARGET Sofa.Component.LinearSolver.Preconditioner)
    include("${CMAKE_CURRENT_LIST_DIR}/Sofa.Component.LinearSolver.PreconditionerTargets.cmake")
endif()

check_required_components(Sofa.Component.LinearSolver.Preconditioner)
