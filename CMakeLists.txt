cmake_minimum_required(VERSION 3.4)
project(TestTOF)
find_package( OpenCV REQUIRED )
include_directories(BEFORE /usr/include/c  /usr/include/cpp ${OpenCV_INCLUDE_DIRS})

add_executable(test_tof test_tof.cpp )
target_link_libraries( test_tof  LINK_PUBLIC sqlite3 ArducamDepthCamera  ${OpenCV_LIBS} )
