#=============================================================================
# CMake configuration file for Chrono-Tutorial
#
#=============================================================================

cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

project(ChronoTutorial)

# Set location of executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

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
option(ENABLE_CHRONO_PROGRAMS "Build the C++ Chrono tutorial programs" ON)
option(ENABLE_VEHICLE_PROGRAMS "Build the C++ Chrono::Vehicle tutorial programs" ON)
option(ENABLE_FEA_PROGRAMS "Build the C++ Chrono::FEA tutorial programs" ON)
option(ENABLE_MULTICORE_PROGRAMS "Build the C++ Chrono::Multicore tutorial programs" ON)
option(ENABLE_SENSOR_PROGRAMS "Build the C++ Chrono::Sensor tutorial programs" ON)
option(ENABLE_SYNCHRONO_PROGRAMS "Build the C++ SynChrono tutorial programs" ON)

# Keep track of all DLLs. Each submodule should append to these list.
list(APPEND ALL_DLL_NAMES "")

# Propagate configuration to submodules.
if(ENABLE_CHRONO_PROGRAMS)
  message(STATUS "\n==== Chrono tutorials ====\n")
 add_subdirectory(Chrono/slider_crank)
endif()
if(ENABLE_VEHICLE_PROGRAMS)
  message(STATUS "\n==== Chrono::Vehicle tutorials ====\n")
  add_subdirectory(Chrono/wheeled_vehicle)
endif()
if(ENABLE_FEA_PROGRAMS)
  message(STATUS "\n==== Chrono::FEA tutorials ====\n")
  add_subdirectory(Chrono/FEA)
endif()
if(ENABLE_MULTICORE_PROGRAMS)
  message(STATUS "\n==== Chrono::Multicore tutorials ====\n")
  add_subdirectory(Chrono/multicore)
endif()
if(ENABLE_SENSOR_PROGRAMS)
  message(STATUS "\n==== Chrono::Sensor tutorials ====\n")
  add_subdirectory(Chrono/sensor)
endif()
if(ENABLE_SYNCHRONO_PROGRAMS)
  message(STATUS "\n==== Chrono::SynChrono tutorials ====\n")
  add_subdirectory(Chrono/synchrono)
endif()

#--------------------------------------------------------------
# Copy DLLs
#--------------------------------------------------------------

# Add custom command for copying DLLs to the appropriate binary output folder
# (no-op if not Windows)
add_CHRONO_DLLS_copy_command("${ALL_DLL_NAMES}")
