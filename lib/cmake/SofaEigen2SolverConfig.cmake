# CMake package configuration file for the SofaEigen2Solver module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaEigen2SolverConfig.cmake.in                            ########

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

find_package(Sofa.Helper QUIET REQUIRED)

get_property(SofaEigen2Solver_SENT_DEPRECATION_MESSAGE GLOBAL PROPERTY PROPERTY_SofaEigen2Solver_SENT_DEPRECATION_MESSAGE SET)
if(NOT SofaEigen2Solver_SENT_DEPRECATION_MESSAGE)
    message(WARNING
        "SofaEigen2Solver has been removed. The package still exists for compatibility but is empty. "
        "Eigen classes were moved to Sofa.LinearAlgebra and SVDLinearSolver to SofaDenseSolver."
        )
endif()
set_property(GLOBAL PROPERTY PROPERTY_SofaEigen2Solver_SENT_DEPRECATION_MESSAGE TRUE)

if(NOT TARGET SofaEigen2Solver)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaEigen2SolverTargets.cmake")
endif()

check_required_components(SofaEigen2Solver)
