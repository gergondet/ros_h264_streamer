cmake_minimum_required(VERSION 2.8.3)
project(ros_h264_streamer)

find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS roscpp image_transport
)

set(ros_h264_streamer_src
  src/h264_encoder.cpp
  src/h264_decoder.cpp
  src/h264_streamer.cpp
  src/h264_receiver.cpp
)

add_library(ros_h264_streamer ${ros_h264_streamer_src})
target_link_libraries(ros_h264_streamer ${catkin_LIBRARIES} x264 avcodec avutil swscale)

add_executable(testEncoderDecoder test/testEncoderDecoder.cpp)
target_link_libraries(testEncoderDecoder ros_h264_streamer ${catkin_LIBRARIES})

add_executable(testStreamer test/testStreamer.cpp)
target_link_libraries(testStreamer ros_h264_streamer ${catkin_LIBRARIES} boost_program_options)

add_executable(testReceiver test/testReceiver.cpp)
target_link_libraries(testReceiver ros_h264_streamer ${catkin_LIBRARIES} boost_program_options)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ros_h264_streamer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
