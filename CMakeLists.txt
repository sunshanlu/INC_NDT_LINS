cmake_minimum_required(VERSION 3.5)
project(inc_ndt_lins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include /usr/inlcude/eigen3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)
add_library(inc_ndt_lins ${SRC_FILES})
target_link_libraries(inc_ndt_lins ${PCL_LIBRARIES} Sophus::Sophus g2o_core
                      g2o_stuff)
ament_target_dependencies(inc_ndt_lins rclcpp sensor_msgs rosbag2_cpp)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
