cmake_minimum_required (VERSION 3.0.0)
project (YOSTLABS-3SPACE-SENSOR)

# enforce build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release CACHE STRING "Setting build mode to Release" FORCE)
endif()

# output location
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin/)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin/)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin/)


# Linux global build options
add_definitions (-DLINUX)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# header search path
include_directories (
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/rapidxml
  ${PROJECT_SOURCE_DIR}/include/serial
  ${PROJECT_SOURCE_DIR}/include/serial/impl
  ${PROJECT_SOURCE_DIR}/include/yost
  )

# static library source files
file (GLOB_RECURSE source RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/*.cpp)
file (GLOB_RECURSE source_yost RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/yost*.cpp)

set (source_serial 
    ${PROJECT_SOURCE_DIR}/src/serial/serial_enumerator.cpp
    ${PROJECT_SOURCE_DIR}/src/serial/serial_enumerator_linux.cpp
    ${PROJECT_SOURCE_DIR}/src/serial/serial_enumerator_osx.cpp
    ${PROJECT_SOURCE_DIR}/src/serial/serial.cc
    ${PROJECT_SOURCE_DIR}/src/serial/impl/unix.cc
     )

# static library
add_library (yostlabs-3space-sensor STATIC 
    ${source} ${source_yost} ${source_serial})

# library exports
set (YOSTLABS_3SPACE_SENSOR_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
set (YOSTLABS_3SPACE_SENSOR_LIBRARIES yostlabs-3space-sensor )
set (YOSTLABS_3SPACE_SENSOR_DEFINITIONS ${PROJECT_DEFINITIONS})
set (YOSTLABS_3SPACE_SENSOR_SOURCE_DIR ${PROJECT_SOURCE_DIR})

# include Eigen
FIND_PACKAGE(Eigen3 REQUIRED)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# executables
include_directories(${PROJECT_SOURCE_DIR}/utils ${EIGEN3_INCLUDE_DIR})
add_executable(3space_redis_driver ${PROJECT_SOURCE_DIR}/3space_redis_driver.cpp ${PROJECT_SOURCE_DIR}/utils/RedisClient.cpp ${PROJECT_SOURCE_DIR}/utils/LoopTimer.cpp)
target_link_libraries(3space_redis_driver ${YOSTLABS_3SPACE_SENSOR_LIBRARIES} -ludev -lpthread ${HIREDIS_LIBRARY})
