cmake_minimum_required(VERSION 3.1)
project(opencv)

find_package(Threads REQUIRED)

set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS regex system thread) # VC require regex, don't know why

find_package(OpenCV REQUIRED COMPONENTS core highgui)

add_executable(opencv opencv.cpp ../common/ai_base.cpp)
include_directories(
  ../common
  ../extlibs/autobahn-cpp
  ../extlibs/msgpack-c/include
  ../extlibs/cppcodec
  ${Boost_INCLUDE_DIRS}
  )
target_link_libraries(opencv
  Threads::Threads
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  )
set_property(TARGET opencv PROPERTY CXX_STANDARD 14)

if(WIN32)
  if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
    include_directories(../extlibs/openssl/include)
  else()
    include_directories(../extlibs/openssl/include64)
  endif()
  add_definitions(-D_WIN32_WINNT=0x0500 -D_WINSOCK_DEPRECATED_NO_WARNINGS)
endif()
