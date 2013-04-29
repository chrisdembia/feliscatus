cmake_minimum_required(VERSION 2.6)

# Define project
SET(PROJ_NAME FelisCatus)
PROJECT(${PROJ_NAME})

################################################################################
# - Pulling in OpenSim.
# Most of this is copied from a CMakeLists.txt somewhere in
# $ENV{OPENSIM_HOME}/sdk/APIExamples/
INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)

SET(USING_SIMBODY_TRUNK FALSE
    CACHE BOOL "If Simbody is built from trunk, then we need different
    libraries.")

SET(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
		CACHE PATH "Top-level directory of OpenSim install")

# To add Debug feature add ";Debug" after Release on the line below
SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release;Debug"
      CACHE STRING "Semicolon separated list of supported configuration types,
      only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything
      else will be ignored." FORCE)

# Location of headers
SET(SIMTK_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
INCLUDE_DIRECTORIES(${SIMTK_HEADERS_DIR} ${OPENSIM_HEADERS_DIR})
# Libraries and dlls
SET(OPENSIM_LIBS_DIR ${OPENSIM_INSTALL_DIR}/sdk/lib ${OPENSIM_INSTALL_DIR}/lib)
SET(OPENSIM_DLLS_DIR ${OPENSIM_INSTALL_DIR}/bin)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR} ${OPENSIM_DLLS_DIR})

IF(WIN32)
    SET(NameSpace "OpenSim_" 
        CACHE STRING "Prefix for simtk lib names, includes trailing '_'. Leave
        empty to use stock SimTK libraries.")
    SET(PLATFORM_LIBS  pthreadVC2)
ELSE(WIN32)
    SET(NameSpace "")
    IF(APPLE)
        SET(PLATFORM_LIBS  SimTKAtlas)
    ELSE(APPLE)
        SET(PLTAFORM_LIBS SimTKAtlas_Lin_generic)
    ENDIF(APPLE)
ENDIF(WIN32)

IF(USING_SIMBODY_TRUNK)
    SET(SIMBODY_LAPACK "")
ELSE(USING_SIMBODY_TRUNK)
    SET(SIMBODY_LAPACK SimTKlapack)
ENDIF(USING_SIMBODY_TRUNK)

# The user is not expected to need to use these variables.
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

SET(OSIM_LIBRARIES
    debug osimSimulation_d    optimized osimSimulation
    debug osimActuators_d    optimized osimActuators
    debug osimCommon_d        optimized osimCommon
    debug osimAnalyses_d    optimized osimAnalyses
    debug osimTools_d        optimized osimTools
    debug ${NameSpace}SimTKcommon_d optimized ${NameSpace}SimTKcommon
    debug ${NameSpace}SimTKmath_d optimized  ${NameSpace}SimTKmath
    debug ${NameSpace}SimTKsimbody_d optimized ${NameSpace}SimTKsimbody
    ${SIMBODY_LAPACK}
    ${PLATFORM_LIBS}
)

################################################################################

# The below is specific to this project.

# EXECUTABLE to make XML template files.
SET(EXECUTABLE_NAME model)
ADD_EXECUTABLE(${EXECUTABLE_NAME} model.cpp)

TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} ${OSIM_LIBRARIES})

# Copy geometry files to the build directory, so OpenSim can find the files
# specified in the model file.
add_custom_command(TARGET ${EXECUTABLE_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/Geometry
    $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)

# Also copy to build top-level directory, because this is where the .osim is placed
# if executable is run directly from Visual Studio.
add_custom_command(TARGET ${EXECUTABLE_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/Geometry
    ${CMAKE_BINARY_DIR})
	
# Copy Geometry files to the OpenSim Geometry folder, so that OpenSim can find
# the geometry files specified in the model file. This is only necessary if we
# plan to move the model file around, and not just use it from the build
# directory.
#FILE(GLOB GEOMETRY_FILES ${CMAKE_SOURCE_DIR}/Geometry/*)
#FOREACH(geometryFile ${GEOMETRY_FILES})
#    ADD_CUSTOM_COMMAND(
        #TARGET ${TARGET}
        #COMMAND ${CMAKE_COMMAND}
        #ARGS -E copy
        #${geometryFile}
        #$ENV{OPENSIM_HOME}/Geometry)
#ENDFOREACH(geometryFile)

#FILE(GLOB DATA_FILES ${CMAKE*.obj)
#FOREACH (dataFile ${DATA_FILES})
# ADD_CUSTOM_COMMAND(
    #TARGET ${TARGET}
    #COMMAND ${CMAKE_COMMAND}
    #ARGS -E copy
    #${dataFile}
    #${OpenSimTugOfWar_BINARY_DIR})
 #ENDFOREACH (dataFile) 
