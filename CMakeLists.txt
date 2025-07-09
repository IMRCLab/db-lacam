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

add_subdirectory(dynoplan)
# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")


# db-pibt
add_library(db-pibt
  ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/src/dbastar/heuristics.cpp
)

target_include_directories(db-pibt
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OMPL_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
    ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/deps/nigh/src
    ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/dynobench/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


target_link_libraries(db-pibt PRIVATE 
  Eigen3::Eigen
  idbastar::tdbastar 
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ${FCL_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${LZ4_LIBRARIES}
)

# main dbpibt
add_executable(
  run_dbpibt
  src/run_dbpibt.cpp 
  src/db-pibt2.cpp
  src/map.cpp
)

target_include_directories(
  run_dbpibt
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${OMPL_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/deps/nigh/src
          ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/dynobench/include
          ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
          ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/dynobench/deps/json/include)

target_link_libraries(run_dbpibt PRIVATE
  fcl 
  idbastar::tdbastar
  ${OMPL_LIBRARIES} 
  ${LZ4_LIBRARIES}
  PUBLIC Eigen3::Eigen)

# db_cbs
add_executable(db_cbs
  src/db_cbs.cpp
)

target_include_directories(db_cbs PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/dynoplan/include
)

target_link_libraries(db_cbs PRIVATE 
  idbastar::optimization
  idbastar::tdbastar
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)