set(TARGET_NAME driver_t265)

add_library(${TARGET_NAME} SHARED
  driverlog.cpp
  driverlog.h
  driver_t265.cpp
)

add_definitions(-DDRIVER_t265_EXPORTS)

include_directories(include ${REALSENSE2_INCLUDE_DIR})
target_include_directories(${TARGET_NAME} PRIVATE ${REALSENSE2_INCLUDE_DIR})

target_link_libraries(${TARGET_NAME}
  ${OPENVR_LIBRARIES}
  ${REALSENSE2_LIBRARY}
  ${CMAKE_DL_LIBS}
)

# Force output directory destination, especially for MSVC (@so7747857).
function(setTargetOutputDirectory target)
  foreach(type RUNTIME LIBRARY ARCHIVE)
    set_target_properties(${target} PROPERTIES
      ${type}_OUTPUT_DIRECTORY         ${CMAKE_${type}_OUTPUT_DIRECTORY}/t265
      ${type}_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_${type}_OUTPUT_DIRECTORY}/t265
      ${type}_OUTPUT_DIRECTORY_RELEASE ${CMAKE_${type}_OUTPUT_DIRECTORY}/t265
    )
  endforeach()
endfunction()

setTargetOutputDirectory(${TARGET_NAME})
