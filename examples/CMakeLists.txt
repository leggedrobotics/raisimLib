cmake_minimum_required(VERSION 3.5)
project(raisim_examples LANGUAGES CXX)

#==
# Dependencies
#==

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

#==
# Build Targets
#==

add_executable(anymal_speed_test anymal.cpp)
target_link_libraries(anymal_speed_test raisim::raisim tcmalloc)
target_compile_definitions(anymal_speed_test PRIVATE "-DRAISIM_EXAMPLE_RESOURCES=${CMAKE_CURRENT_LIST_DIR}")
target_include_directories(anymal_speed_test PUBLIC ${EIGEN3_INCLUDE_DIR})
