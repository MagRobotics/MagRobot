# CMake package configuration file for the  plugin


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SoftRobotsConfig.cmake.in                            ########

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

set(SOFTROBOTS_HAVE_SOFAPYTHON )
set(SOFTROBOTS_HAVE_SOFAPYTHON3 )
set(SOFTROBOTS_HAVE_STLIB 0)
set(SOFTROBOTS_HAVE_SOFA_GL 1)

find_package(SofaFramework QUIET REQUIRED)
find_package(SofaUserInteraction QUIET REQUIRED)
find_package(SofaBaseMechanics QUIET REQUIRED)
find_package(SofaConstraint QUIET REQUIRED)
find_package(SofaRigid QUIET REQUIRED)
find_package(SofaMiscMapping QUIET REQUIRED)
find_package(SofaEngine QUIET REQUIRED)
find_package(SofaSparseSolver QUIET REQUIRED)
find_package(Qt5 QUIET REQUIRED COMPONENTS Network)

if(SOFTROBOTS_HAVE_SOFAPYTHON)
    find_package(SofaPython QUIET REQUIRED)
endif()
if(SOFTROBOTS_HAVE_SOFAPYTHON3)
    find_package(SofaPython3 QUIET REQUIRED)
endif()
if(SOFTROBOTS_HAVE_STLIB)
    find_package(STLIB QUIET REQUIRED)
endif()
if(SOFTROBOTS_HAVE_SOFA_GL)
    find_package(Sofa.GL QUIET REQUIRED)
endif()

if(NOT TARGET SoftRobots)
    include("${CMAKE_CURRENT_LIST_DIR}/SoftRobotsTargets.cmake")
endif()

check_required_components(SoftRobots)

