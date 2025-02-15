cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(DBSCAN_CLUSTERING)

# Find PCL
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find OpenMP
find_package(OpenMP REQUIRED)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)

# Create the executable
add_executable(dbscan_clustering example.cpp dbscan.cpp)

# Link PCL libraries
target_link_libraries(dbscan_clustering PUBLIC ${PCL_LIBRARIES})

# If OpenMP is found, link it as well
if(OpenMP_CXX_FOUND)
    target_compile_options(dbscan_clustering PUBLIC ${OpenMP_CXX_FLAGS})
    target_link_libraries(dbscan_clustering PUBLIC ${OpenMP_CXX_LIBRARIES})
endif()
