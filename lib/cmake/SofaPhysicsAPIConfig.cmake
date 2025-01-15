# CMake package configuration file for SofaPhysicsAPI


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaPhysicsAPIConfig.cmake.in                            ########

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

check_required_components(SofaPhysicsAPI)

find_package(GLUT REQUIRED)
find_package(SofaGui REQUIRED)

# Are all these modules (formerly SofaGeneral) necessary?
find_package(SofaBoundaryCondition REQUIRED)
find_package(SofaConstraint REQUIRED)
find_package(SofaGeneralAnimationLoop REQUIRED)
find_package(SofaGeneralDeformable REQUIRED)
find_package(SofaGeneralEngine REQUIRED)
find_package(SofaGeneralExplicitOdeSolver REQUIRED)
find_package(SofaGeneralImplicitOdeSolver REQUIRED)
find_package(SofaGeneralLinearSolver REQUIRED)
find_package(SofaGeneralLoader REQUIRED)
find_package(SofaGeneralMeshCollision REQUIRED)
find_package(SofaGeneralObjectInteraction REQUIRED)
find_package(SofaGeneralRigid REQUIRED)
find_package(SofaGeneralSimpleFem REQUIRED)
find_package(SofaGeneralTopology REQUIRED)
find_package(SofaGeneralVisual REQUIRED)
find_package(SofaGraphComponent REQUIRED)
find_package(SofaTopologyMapping REQUIRED)
find_package(SofaUserInteraction REQUIRED)

find_package(SceneCreator REQUIRED)
find_package(SofaValidation REQUIRED)
find_package(SofaSimulationCommon REQUIRED)

if(NOT TARGET SofaPhysicsAPI)
	include("${CMAKE_CURRENT_LIST_DIR}/SofaPhysicsAPITargets.cmake")
endif()
