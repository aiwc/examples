cmake_minimum_required(VERSION 3.1)
project(random_walk)

find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(random_walk random_walk.cpp ../common/ai_base.cpp)
include_directories(
  ../common
  ../extlibs/autobahn-cpp
  ../extlibs/msgpack-c/include
  ${Boost_INCLUDE_DIRS}
  )
target_link_libraries(random_walk
  Threads::Threads
  ${Boost_LIBRARIES}
  )
set_property(TARGET random_walk PROPERTY CXX_STANDARD 14)
