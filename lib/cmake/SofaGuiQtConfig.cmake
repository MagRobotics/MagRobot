# CMake package configuration file for the SofaGuiQt module

### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was SofaGuiQtConfig.cmake.in                            ########

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

set(SOFAGUIQT_HAVE_QT5 1)
set(SOFAGUIQT_HAVE_QT6 )

set(SOFAGUIQT_HAVE_QTVIEWER 1)
set(SOFAGUIQT_HAVE_QGLVIEWER 1)
set(SOFAGUIQT_HAVE_QT5_CHARTS 1)
set(SOFAGUIQT_HAVE_QT5_WEBENGINE 1)
set(SOFAGUIQT_HAVE_NODEEDITOR )
set(SOFAGUIQT_HAVE_TINYXML )

# This is needed to be able to find plugins/SofaGuiQt/lib/cmake/QGLViewer
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/..")


find_package(SofaBaseVisual QUIET REQUIRED)
find_package(SofaLoader QUIET REQUIRED)
find_package(SofaMiscForceField QUIET REQUIRED)
find_package(SofaGuiCommon QUIET REQUIRED)

if(SOFAGUIQT_HAVE_QT5)
    find_package(Qt5 QUIET REQUIRED Core Gui OpenGL)
    if(SOFAGUIQT_HAVE_QT5_CHARTS)
        find_package(Qt5 QUIET REQUIRED Charts)
    endif()
    if(SOFAGUIQT_HAVE_QT5_WEBENGINE)
        find_package(Qt5 QUIET REQUIRED WebEngine WebEngineWidgets)
    endif()
    if(SOFAGUIQT_HAVE_QGLVIEWER)
        find_package(QGLViewer QUIET REQUIRED)
    endif()
elseif(SOFAGUIQT_HAVE_QT6)
    find_package(Qt6 COMPONENTS Widgets OpenGLWidgets QUIET REQUIRED)
endif()

if(SOFAGUIQT_HAVE_NODEEDITOR)
    find_package(NodeEditor QUIET REQUIRED)
endif()
if(SOFAGUIQT_HAVE_TINYXML)
    find_package(TinyXML QUIET REQUIRED)
endif()

if(NOT TARGET SofaGuiQt)
    include("${CMAKE_CURRENT_LIST_DIR}/SofaGuiQtTargets.cmake")
endif()

check_required_components(SofaGuiQt)
