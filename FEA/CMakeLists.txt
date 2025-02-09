set(MODELS
  FEA_cable_collide_1
  FEA_cable_collide_2
  FEA_cable_collide_3
  FEA_cable_collide_1_solution
  FEA_cable_collide_2_solution
  FEA_cable_collide_3_solution
  )

#--------------------------------------------------------------
# Find the Chrono package with required components
#--------------------------------------------------------------

# Invoke find_package in CONFIG mode
find_package(Chrono
             COMPONENTS Irrlicht
             CONFIG
)

# If Chrono and/or the required component(s) were not found, return now.
if(NOT Chrono_FOUND)
  message("Could not find requirements for Chrono tutorials")
  return()
endif()

#--------------------------------------------------------------
# Append to the parent's lists of DLLs (and make them visible)
#--------------------------------------------------------------

list(APPEND ALL_DLL_NAMES       "${CHRONO_DLL_NAMES}")
list(APPEND ALL_DEPENDENCY_DLLS "${CHRONO_DEPENDENCY_DLLS}")
set(ALL_DLL_NAMES       "${ALL_DLL_NAMES}"       PARENT_SCOPE)
set(ALL_DEPENDENCY_DLLS "${ALL_DEPENDENCY_DLLS}" PARENT_SCOPE)

#--------------------------------------------------------------
# Compilation flags
#--------------------------------------------------------------

foreach(PROGRAM ${MODELS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES "${PROGRAM}.cpp")

  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

  target_compile_definitions(${PROGRAM} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
  target_link_libraries(${PROGRAM} PRIVATE ${CHRONO_TARGETS})
endforeach()
