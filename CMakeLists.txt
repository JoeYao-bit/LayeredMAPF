cmake_minimum_required(VERSION 2.8)

project(LayeredMAPF)
enable_testing()
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

add_subdirectory(third_party/grid-pathfinding/graph ./graph)


add_subdirectory(freeNav-base)
include_directories(third_party)

aux_source_directory(third_party/EECBS/src                  EECBS_src)
aux_source_directory(third_party/CBSH2-RTC/src              CBSH2-RTC_src)
aux_source_directory(third_party/PBS/src                    PBS_src)
aux_source_directory(third_party/lacam/src                  lacam_src)
aux_source_directory(third_party/lacam2/src                 lacam2_src)
aux_source_directory(third_party/MAPF-LNS2/src              MAPF_LNS2_src)
aux_source_directory(third_party/MAPF-LNS2/src/CBS          MAPF_LNS2_CBS_src)
aux_source_directory(third_party/MAPF-LNS2/src/PIBT         MAPF_LNS2_PIBT_src)
aux_source_directory(third_party/pibt2/src                  PIBT2_src)

add_library(EECBS_lib  ${EECBS_src} ${PBS_src} ${lacam_src} ${CBSH2-RTC_src} ${lacam2_src}
        ${MAPF_LNS2_src} ${MAPF_LNS2_CBS_src} ${MAPF_LNS2_PIBT_src} ${PIBT2_src} )

target_link_libraries(EECBS_lib freeNav-base)

aux_source_directory(algorithm alg_src)
aux_source_directory(algorithm/LA-MAPF lamapf_src)
add_library(alg_lib ${alg_src} ${lamapf_src})
target_link_libraries(alg_lib EECBS_lib lib-graph)
#add_library(external_lib ${third1_src} ${third2_src} ${third3_src})

add_executable(test_single_layered_MAPF test/test_single_layered_MAPF.cpp)
target_link_libraries(test_single_layered_MAPF alg_lib)

add_executable(test_memory_check test/test_massive_decomposition.cpp)
target_link_libraries(test_memory_check alg_lib)

add_executable(test_massive_layered_mapf test/test_massive_layered_mapf.cpp)
target_link_libraries(test_massive_layered_mapf alg_lib)

add_executable(test_geometry test/test_geometry.cpp)
target_link_libraries(test_geometry alg_lib)

