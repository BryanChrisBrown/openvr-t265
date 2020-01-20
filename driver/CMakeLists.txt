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

setTargetOutputDirectory(${TARGET_NAME})
