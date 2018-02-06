project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
  src/main.cpp
  src/behavior.cpp
  src/behavior.hpp
  src/path_common.cpp
  src/path_common.hpp
  src/prediction.cpp
  src/prediction.hpp
  src/sensor_fusion.cpp
  src/sensor_fusion.hpp
  src/trajectory.cpp
  src/trajectory.hpp
  src/vehicle.cpp
  src/vehicle.hpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
