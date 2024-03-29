cmake_minimum_required (VERSION 3.0.0)
project (localization)

# output location
 set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin/)
 set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin/)
 set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin/)
 set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin/)
 set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin/)

set(CMAKE_BUILD_TYPE Debug)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

# Linux global build options
add_definitions (-DLINUX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/support/cmake")

#include(cxx14)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_subdirectory(src/external/libsurvive)
add_subdirectory(src/external/robomaster_sdk_can)

find_path(REDIS_PLUS_PLUS_HEADER sw)

# header search path
include_directories (
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/rapidxml
  ${PROJECT_SOURCE_DIR}/include/serial
  ${PROJECT_SOURCE_DIR}/include/serial/impl
  ${PROJECT_SOURCE_DIR}/include/yost
  ${PROJECT_SOURCE_DIR}/src/external/libsurvive/include
  ${PROJECT_SOURCE_DIR}/src/external/libsurvive/include/libsurvive
  ${PROJECT_SOURCE_DIR}/src/external/libsurvive/libs/cnkalman/include
  ${PROJECT_SOURCE_DIR}/src/external/libsurvive/libs/cnkalman/libs/cnmatrix/include
  ${PROJECT_SOURCE_DIR}/src/external/libsurvive/redist
  ${PROJECT_SOURCE_DIR}/src/external/robomaster_sdk_can/inc
  ${REDIS_PLUS_PLUS_HEADERS}
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

option(WITH_ZED "Zed camera localization" OFF)

if(WITH_ZED)
  # zed camera
  find_package(ZED 3 REQUIRED)
  find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
endif()

# include Eigen
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

find_package(Threads REQUIRED)

find_package(msgpack REQUIRED)

find_package(wpilib REQUIRED)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

find_library(REDIS_PLUS_PLUS_LIB redis++)


# http client
include(FetchContent)
FetchContent_Declare(cpr GIT_REPOSITORY https://github.com/libcpr/cpr.git
                         GIT_TAG db351ffbbadc6c4e9239daaa26e9aefa9f0ec82d) # 1.8.3, from https://github.com/libcpr/cpr/releases
FetchContent_MakeAvailable(cpr)

# executables
include_directories(${PROJECT_SOURCE_DIR}/utils ${EIGEN3_INCLUDE_DIR})

if(WITH_ZED)
  include_directories(${ZED_INCLUDE_DIRS})
  include_directories(${CUDA_INCLUDE_DIRS})

  link_directories(${ZED_LIBRARY_DIR})
  link_directories(${CUDA_LIBRARY_DIRS})

  if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
  else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
  endif()
endif()

add_executable(mecanum_drive_controller ${PROJECT_SOURCE_DIR}/mecanum_drive_controller.cpp ${PROJECT_SOURCE_DIR}/utils/LoopTimer.cpp)

add_executable(differential_drive_controller ${PROJECT_SOURCE_DIR}/differential_drive_controller.cpp ${PROJECT_SOURCE_DIR}/utils/LoopTimer.cpp)

add_executable(3space_redis_driver ${PROJECT_SOURCE_DIR}/3space_redis_driver.cpp ${PROJECT_SOURCE_DIR}/utils/LoopTimer.cpp)

if(WITH_ZED)
  add_executable(zed_localization ${PROJECT_SOURCE_DIR}/zed_localization.cpp)
endif()

add_executable(survive_redis_driver ${PROJECT_SOURCE_DIR}/survive_redis_driver.cpp)

add_executable(gm6020_can_redis_driver ${PROJECT_SOURCE_DIR}/gm6020_can_redis_driver.cpp)

add_executable(robomaster_can_redis_driver ${PROJECT_SOURCE_DIR}/robomaster_can_redis_driver.cpp)

set_target_properties(gm6020_can_redis_driver PROPERTIES CXX_STANDARD 17)

set_target_properties(mecanum_drive_controller PROPERTIES CXX_STANDARD 17)

set_target_properties(differential_drive_controller PROPERTIES CXX_STANDARD 17)

set_target_properties(3space_redis_driver PROPERTIES CXX_STANDARD 17)

if(WITH_ZED)
  set_target_properties(zed_localization PROPERTIES CXX_STANDARD 17)
endif()

set_target_properties(survive_redis_driver PROPERTIES CXX_STANDARD 17)

set_target_properties(robomaster_can_redis_driver PROPERTIES CXX_STANDARD 17)

target_link_libraries(mecanum_drive_controller wpilibc wpiutil wpimath ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} Eigen3::Eigen -lmsgpackc cpr::cpr)

target_link_libraries(differential_drive_controller wpilibc wpiutil wpimath ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} Eigen3::Eigen -lmsgpackc cpr::cpr)

target_link_libraries(3space_redis_driver ${YOSTLABS_3SPACE_SENSOR_LIBRARIES} -ludev -lpthread ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} -lmsgpackc)

if(WITH_ZED)
  target_link_libraries(zed_localization ${ZED_LIBS} -ludev -lpthread ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} -lmsgpackc)
endif()

target_link_libraries(survive_redis_driver survive -ludev -lpthread ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} -lmsgpackc)

target_link_libraries(robomaster_can_redis_driver -ludev -lpthread ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} Threads::Threads -lmsgpackc)

target_link_libraries(gm6020_can_redis_driver -ludev -lpthread ${HIREDIS_LIBRARY} ${REDIS_PLUS_PLUS_LIB} Threads::Threads -lmsgpackc)
