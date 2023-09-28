#=============================================================================
# CMake configuration file for Chrono-Tutorial
#
#=============================================================================

cmake_minimum_required(VERSION 3.10)

project(ChronoTutorial)

# Set location of executable
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)


#-----------------------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)
#-----------------------------------------------------------------------------

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

#-----------------------------------------------------------------------------
# Force C++11
#-----------------------------------------------------------------------------

#if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
#endif()

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#-----------------------------------------------------------------------------
# Disable some warnings
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-DNOMINMAX" ) # do not use MSVC's min/max macros
        add_definitions( "-MP" ) # perform parallel builds
        add_compile_options(/wd4251) # class needs to have dll-interface
    endif(MSVC)
endif()

#-----------------------------------------------------------------------------
# Invoke CMake in subdirectories
#-----------------------------------------------------------------------------

if(NOT Chrono_DIR)
  set(Chrono_DIR "" CACHE PATH "The directory containing a CMake configuration file for Chrono.")
  return()
endif()

# Options for configuring/building individual sets of programs
option(ENABLE_CHRONO_PROGRAMS "Build the Chrono tutorial programs" ON)
option(ENABLE_VEHICLE_PROGRAMS "Build the Chrono::Vehicle tutorial programs" ON)
option(ENABLE_FEA_PROGRAMS "Build the Chrono::FEA tutorial programs" ON)
option(ENABLE_MULTICORE_PROGRAMS "Build the Chrono::Multicore tutorial programs" ON)
option(ENABLE_SENSOR_PROGRAMS "Build the Chrono::Sensor tutorial programs" OFF)
option(ENABLE_SYNCHRONO_PROGRAMS "Build the SynChrono tutorial programs" OFF)

# Keep track of all DLLs. Each submodule should append to these list.
list(APPEND ALL_DLL_NAMES "")
list(APPEND ALL_DEPENDENCY_DLLS "")

# Propagate configuration to submodules.
if(ENABLE_CHRONO_PROGRAMS)
  message(STATUS "==== Chrono programs ====")
 add_subdirectory(slider_crank)
endif()
if(ENABLE_VEHICLE_PROGRAMS)
  message(STATUS "==== Chrono::Vehicle programs ====")
  add_subdirectory(wheeled_vehicle)
endif()
if(ENABLE_FEA_PROGRAMS)
  message(STATUS "==== Chrono::FEA programs ====")
  add_subdirectory(FEA)
endif()
if(ENABLE_MULTICORE_PROGRAMS)
  message(STATUS "==== Chrono::Multicore programs ====")
  add_subdirectory(multicore)
endif()
if(ENABLE_SENSOR_PROGRAMS)
  message(STATUS "==== Chrono::Sensor programs ====")
  add_subdirectory(sensor)
endif()
if(ENABLE_SYNCHRONO_PROGRAMS)
  message(STATUS "==== SynChrono programs ====")
  add_subdirectory(synchrono)
endif()

#--------------------------------------------------------------
# Copy DLLs
#--------------------------------------------------------------

# Add custom command for copying DLLs to the appropriate binary
# output folder (on Windows only).

add_DLL_copy_command("${ALL_DLL_NAMES}" "${ALL_DEPENDENCY_DLLS}")
