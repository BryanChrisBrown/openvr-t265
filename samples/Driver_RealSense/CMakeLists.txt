set(TARGET_NAME driver_t265)

add_library(${TARGET_NAME} SHARED
  driverlog.cpp
  driverlog.h
  driver_t265.cpp
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "C:\Program Files (x86)\Intel RealSense SDK 2.0\include")

find_package(realsense2 REQUIRED)

add_definitions(-DDRIVER_t265_EXPORTS)

include_directories(include ${realsense_INCLUDE_DIR})
target_include_directories(${TARGET_NAME} PRIVATE ${realsense_INCLUDE_DIR})

target_link_libraries(${TARGET_NAME}
  ${OPENVR_LIBRARIES}
  ${realsense2_LIBRARY}
  ${CMAKE_DL_LIBS}
)

setTargetOutputDirectory(${TARGET_NAME})
