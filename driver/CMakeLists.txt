set(TARGET_NAME openvr-zedm)

add_library(${TARGET_NAME} SHARED
  driver_zedm.cpp
  driverlog.cpp
  driverlog.h
)

add_definitions(-DDRIVER_ZEDM_EXPORTS)

include_directories(include ${ZED_INCLUDE_DIR})
target_include_directories(${TARGET_NAME} PRIVATE ${ZED_INCLUDE_DIR})

target_link_libraries(${TARGET_NAME}
  ${OPENVR_LIBRARIES}
  ${ZED_LIBRARY}
  ${CMAKE_DL_LIBS}
)

# Force output directory destination, especially for MSVC (@so7747857).
function(setTargetOutputDirectory target)
  foreach(type RUNTIME LIBRARY ARCHIVE)
    set_target_properties(${target} PROPERTIES
      ${type}_OUTPUT_DIRECTORY         ${CMAKE_HOME_DIRECTORY}/bin/zedm/bin/${ARCH_TARGET}
      ${type}_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_HOME_DIRECTORY}/bin/zedm/bin/${ARCH_TARGET}
      ${type}_OUTPUT_DIRECTORY_RELEASE ${CMAKE_HOME_DIRECTORY}/bin/zedm/bin/${ARCH_TARGET}
    )
  endforeach()
endfunction()

setTargetOutputDirectory(${TARGET_NAME})
