cmake_minimum_required(VERSION 3.5)

project(db-lacam)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)
find_package(ompl REQUIRED)
message(STATUS "OMPL_FOUND: ${OMPL_FOUND}")
message(STATUS "OMPL_INCLUDE_DIRS: ${OMPL_INCLUDE_DIRS}")
message(STATUS "OMPL_LIBRARIES: ${OMPL_LIBRARIES}")
message(STATUS "OMPL_VERSION: ${OMPL_VERSION}")

find_library(
  LZ4_LIBRARIES
  NAMES liblz4.so
  HINTS "lz4/lib")
if(LZ4_LIBRARIES)
  message(STATUS "Found: ${LZ4_LIBRARIES}")
else()
  message(STATUS "Not found: ${LZ4_LIBRARIES}")
  message(FATAL_ERROR "Cannot find required LZ4 libraries")
endif()

add_subdirectory(dynobench)

# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# tdbastar
add_library(tdbastar src/tdbastar.cpp 
                     src/db_options.cpp src/heuristics.cpp
                     src/motion.cpp)

target_include_directories(
  tdbastar
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_link_libraries(
  tdbastar
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

# db-pibt
add_library(db-pibt
  src/db-pibt.cpp
  src/heuristics.cpp
  src/db_options.cpp
  src/heuristics.cpp
  src/motion.cpp
)

target_include_directories(db-pibt PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynobench/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(db-pibt PRIVATE 
  PUBLIC Eigen3::Eigen dynobench::dynobench
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ${FCL_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${LZ4_LIBRARIES}
)

# db-pibt recursive with priority inheritance
add_library(db-pibt2
  src/db-pibt2.cpp
  src/heuristics.cpp
  src/db_options.cpp
  src/heuristics.cpp
  src/motion.cpp
  # src/map.cpp
)

target_include_directories(db-pibt2 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynobench/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(db-pibt2 PRIVATE 
  PUBLIC Eigen3::Eigen dynobench::dynobench
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ${FCL_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${LZ4_LIBRARIES}
)

# test-pibt
add_executable(
  test-dbpibt
  src/test-dbpibt.cpp src/db-pibt.cpp src/tdbastar.cpp
  src/heuristics.cpp
  src/db_options.cpp
  src/motion.cpp
)

target_include_directories(
  test-dbpibt
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_link_libraries(
  test-dbpibt
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})

  # check the map used for occupancy tracking
#   add_executable(occupancy_map
#   src/map.cpp
# )

# add_library(map src/map.cpp)

# main dbpibt
add_executable(
  main-dbpibt
  src/main-dbpibt.cpp src/db-pibt.cpp src/db-pibt2.cpp
  src/tdbastar.cpp
  src/heuristics.cpp
  src/db_options.cpp
  src/motion.cpp
  src/map.cpp
)

target_include_directories(
  main-dbpibt
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}/deps/nigh/src)

target_link_libraries(
  main-dbpibt
  PUBLIC Eigen3::Eigen dynobench::dynobench
  PRIVATE fcl ${OMPL_LIBRARIES} ${LZ4_LIBRARIES})
