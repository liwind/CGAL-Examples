cmake_minimum_required(VERSION 3.20)
project(CGALExamples)

# set output locations
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type to Release or Debug.")
endif()
message(STATUS "cmake build type: ${CMAKE_BUILD_TYPE}")

# set Boost multiprecision backend
set(CGAL_CMAKE_EXACT_NT_BACKEND BOOST_BACKEND CACHE STRING "Boost multiprecision backend.")
set(CGAL_DISABLE_GMP ON CACHE BOOL "")
set(CMAKE_DISABLE_FIND_PACKAGE_GMP ON CACHE BOOL "")

# Boost and its components
find_package(Boost REQUIRED)
if (NOT Boost_FOUND)
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()
# CGAL and its components
find_package(CGAL REQUIRED)
find_package(Eigen3 3.3)
include(CGAL_Eigen3_support)
if(NOT TARGET CGAL::Eigen3_support)
  message(STATUS "This project requires the Eigen library, and will not be compiled.")
  return()
endif()
# add polyscope library
add_subdirectory(deps/polyscope)

# include for local directory
#include_directories(BEFORE polyscope_path)

# Creating entries for all C++ files with "main" routine
# create_single_source_cgal_program("heat_method.cpp")
add_executable(11points_and_segment src/11points_and_segment.cpp)
target_link_libraries(11points_and_segment PUBLIC CGAL::CGAL)

add_executable(12surprising src/12surprising.cpp)
target_link_libraries(12surprising PUBLIC CGAL::CGAL)

add_executable(13exact src/13exact.cpp)
target_link_libraries(13exact PUBLIC CGAL::CGAL)

add_executable(14array_convex_hull_2 src/14array_convex_hull_2.cpp)
target_link_libraries(14array_convex_hull_2 PUBLIC CGAL::CGAL)

add_executable(15vector_convex_hull_2 src/15vector_convex_hull_2.cpp)
target_link_libraries(15vector_convex_hull_2 PUBLIC CGAL::CGAL)

add_executable(16convex_hull_yz src/16convex_hull_yz.cpp)
target_link_libraries(16convex_hull_yz PUBLIC CGAL::CGAL)

add_executable(21surface_mesh src/21surface_mesh.cpp)
target_link_libraries(21surface_mesh PUBLIC CGAL::CGAL CGAL::Eigen3_support)

add_executable(22mesh_curvature src/22mesh_curvature.cpp)
target_link_libraries(22mesh_curvature PUBLIC polyscope CGAL::CGAL CGAL::Eigen3_support)
target_include_directories(22mesh_curvature BEFORE PUBLIC ${CMAKE_SOURCE_DIR}/src)