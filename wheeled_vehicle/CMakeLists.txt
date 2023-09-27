
SET(MODELS
  wheeled_vehicle
  wheeled_vehicle_solution
  )

#--------------------------------------------------------------
# Find the Chrono package with required components
#--------------------------------------------------------------

# Invoke find_package in CONFIG mode
find_package(Chrono
             COMPONENTS Vehicle Irrlicht
             CONFIG
)

# If Chrono and/or the required component(s) were not found, return now.
if(NOT Chrono_FOUND)
  message("Could not find requirements for Chrono::Vehicle tutorials")
  return()
endif()

# Use same C++ standard as for building Chrono
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

include_directories(${CHRONO_INCLUDE_DIRS})

#--------------------------------------------------------------
# Append to the parent's list of DLLs (and make it visible up)
#--------------------------------------------------------------

list(APPEND ALL_DLLS "${CHRONO_DLLS}")
set(ALL_DLLS "${ALL_DLLS}" PARENT_SCOPE)

#--------------------------------------------------------------
# Compilation flags
#--------------------------------------------------------------

# Disable some warnings triggered by Irrlicht (Windows only)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  add_compile_options(/wd4251)
endif()

foreach(PROGRAM ${MODELS})

  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES "${PROGRAM}.cpp")

  target_compile_definitions(${PROGRAM} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
  target_compile_definitions(${PROGRAM} PUBLIC "SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
  target_compile_options(${PROGRAM} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${PROGRAM} PUBLIC ${CHRONO_LINKER_FLAGS})
  target_link_libraries(${PROGRAM} ${CHRONO_LIBRARIES})

endforeach()
