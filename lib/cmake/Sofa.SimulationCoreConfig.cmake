# CMake package configuration file for the Sofa.SimulationCore module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Sofa.SimulationCoreConfig.cmake.in                            ########

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

get_property(Sofa.SimulationCore_SENT_DEPRECATION_MESSAGE GLOBAL PROPERTY PROPERTY_Sofa.SimulationCore_SENT_DEPRECATION_MESSAGE SET)
if(NOT Sofa.SimulationCore_SENT_DEPRECATION_MESSAGE)
    message(WARNING "${PROJECT_NAME} module is deprecated. It will be removed at v23.06. Use Sofa.Simulation.Core instead.")
endif()
set_property(GLOBAL PROPERTY PROPERTY_Sofa.SimulationCore_SENT_DEPRECATION_MESSAGE TRUE)

# Create alias to support compatibility, starting from v21.06
# This alias will be deleted in v22.06
if(NOT TARGET SofaSimulationCore) # test if alias was not already created
    get_target_property(Sofa.SimulationCore_IMPORTED Sofa.SimulationCore IMPORTED)
    if(Sofa.SimulationCore_IMPORTED)
        set_target_properties(Sofa.SimulationCore PROPERTIES IMPORTED_GLOBAL TRUE)
    endif()
    add_library(SofaSimulationCore ALIAS Sofa.SimulationCore)
endif()

find_package(Sofa.Simulation.Core QUIET REQUIRED)
