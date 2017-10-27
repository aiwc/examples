cmake_minimum_required(VERSION 3.1)
project(skeleton)

find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(skeleton skeleton.cpp ../common/ai_base.cpp)
include_directories(
  ../common
  ../extlibs/autobahn-cpp
  ../extlibs/msgpack-c/include
  ${Boost_INCLUDE_DIRS}
  )
target_link_libraries(skeleton
  Threads::Threads
  ${Boost_LIBRARIES}
  )
set_property(TARGET skeleton PROPERTY CXX_STANDARD 14)
