cmake_minimum_required(VERSION 2.6)

# Define project
SET(PROJ_NAME FelisCatus)
PROJECT(${PROJ_NAME})

################################################################################
# - Pulling in OpenSim.
# Most of this is copied from a CMakeLists.txt somewhere in
# $ENV{OPENSIM_HOME}/sdk/APIExamples/
INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)

SET(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
		CACHE PATH "Top-level directory of OpenSim install")

# To add Debug feature add ";Debug" after Release on the line below
SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
      CACHE STRING "Semicolon separated list of supported configuration types,
      only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything
      else will be ignored." FORCE)

# Location of headers
SET(SIMTK_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
INCLUDE_DIRECTORIES(${SIMTK_HEADERS_DIR} ${OPENSIM_HEADERS_DIR})
# Libraries and dlls
SET(OPENSIM_LIBS_DIR ${OPENSIM_INSTALL_DIR}/lib)
SET(OPENSIM_DLLS_DIR ${OPENSIM_INSTALL_DIR}/bin)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR} ${OPENSIM_DLLS_DIR})

IF(WIN32)
	SET(PLATFORM_LIBS  pthreadVC2)
ELSE (WIN32)
  SET(NameSpace "")
  IF(APPLE)
	SET(PLATFORM_LIBS  SimTKAtlas)
  ELSE(APPLE)
	SET(PLTAFORM_LIBS SimTKAtlas_Lin_generic)
  ENDIF(APPLE)
ENDIF (WIN32)

# The user is not expected to need to use these variables.
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

SET(OSIM_LIBRARIES
	debug osimCommon_d optimized osimCommon
	debug osimSimulation_d optimized osimSimulation
	debug osimAnalyses_d optimized osimAnalyses
	debug osimActuators_d optimized osimActuators
	debug osimTools_d optimized osimTools
	debug SimTKcommon_d optimized SimTKcommon
	debug SimTKmath_d optimized SimTKmath
	debug SimTKsimbody_d optimized SimTKsimbody
	${PLATFORM_LIBS}
    )

################################################################################

# The below is specific to this project.

# EXECUTABLE to make XML template files.
SET(EXECUTABLE_NAME model)
ADD_EXECUTABLE(${EXECUTABLE_NAME} model.cpp)

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${OSIM_LIBRARIES})

# Copy necessary files to build directory.
#add_custom_command(TARGET ${EXECUTABLE_NAME}
#    POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#    ${CMAKE_SOURCE_DIR}/input
#    ${CMAKE_BINARY_DIR}/input)
