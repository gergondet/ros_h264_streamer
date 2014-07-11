cmake_minimum_required(VERSION 2.8.3)
project(ros_h264_streamer)

if(NOT WIN32)
find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})
else()
add_definitions("-D_WIN32_WINNT=0x0501")
include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)
set(catkin_LIBRARIES "")
set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
set(CATKIN_PACKAGE_BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
set(CATKIN_PACKAGE_LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(catkin_FOUND 0)
endif()

find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(catkin_FOUND)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS roscpp image_transport
)
endif()

if(NOT WIN32)
set(ros_h264_streamer_src
  src/h264_encoder.cpp
  src/h264_decoder.cpp
  src/h264_streamer.cpp
  src/h264_receiver.cpp
)
else()
set(ros_h264_streamer_src
  src/h264_encoder.cpp
  src/h264_decoder.cpp
  src/h264_receiver.cpp
)
endif()

add_library(ros_h264_streamer SHARED ${ros_h264_streamer_src})
target_link_libraries(ros_h264_streamer ${catkin_LIBRARIES} x264 avcodec avutil swscale)

if(NOT WIN32)
# Test encoder/decoder requires opencv, we don't pull this dependency for win32
add_executable(testEncoderDecoder test/testEncoderDecoder.cpp)
target_link_libraries(testEncoderDecoder ros_h264_streamer ${catkin_LIBRARIES})
#Obviously streamer relies way too much on ROS
add_executable(testStreamer test/testStreamer.cpp)
target_link_libraries(testStreamer ros_h264_streamer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()

add_executable(testReceiver test/testReceiver.cpp)
if(NOT WIN32)
target_link_libraries(testReceiver ros_h264_streamer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
else()
target_link_libraries(testReceiver ros_h264_streamer)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(NOT WIN32)
set(SHOULD_INSTALL ros_h264_streamer testEncoderDecoder testStreamer testReceiver)
else()
set(SHOULD_INSTALL ros_h264_streamer testReceiver)
endif()

install(TARGETS ${SHOULD_INSTALL}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
