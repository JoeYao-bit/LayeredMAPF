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
aux_source_directory(third_party/lacam3/src                 lacam3_src)
aux_source_directory(third_party/MAPF-LNS2/src              MAPF_LNS2_src)
aux_source_directory(third_party/MAPF-LNS2/src/CBS          MAPF_LNS2_CBS_src)
aux_source_directory(third_party/MAPF-LNS2/src/PIBT         MAPF_LNS2_PIBT_src)
aux_source_directory(third_party/pibt2/src                  PIBT2_src)
aux_source_directory(third_party/Hybrid_MAPF                Hybird_MAPF_src)
aux_source_directory(algorithm alg_src)
aux_source_directory(algorithm/LA-MAPF lamapf_src)
aux_source_directory(algorithm/constraint_table_CBS constraint_table_src)

add_library(alg_lib  ${EECBS_src} ${PBS_src} ${lacam_src} ${CBSH2-RTC_src} ${lacam2_src} ${lacam3_src}
        ${MAPF_LNS2_src} ${MAPF_LNS2_CBS_src} ${MAPF_LNS2_PIBT_src} ${PIBT2_src} ${Hybird_MAPF_src} ${alg_src} ${lamapf_src} ${constraint_table_src})



add_library(lamapf_alg STATIC ${alg_src} ${lamapf_src})

#add_library(lamapf_alg_ex STATIC algorithm/LA-MAPF/block_shaped_agent.cpp
#                                 algorithm/LA-MAPF/circle_shaped_agent.cpp
#                                 algorithm/LA-MAPF/common.cpp)
#INSTALL (TARGETS lamapf_alg_ex lamapf_alg_ex LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

target_link_libraries(lamapf_alg alg_lib lib-graph freeNav-base)

#INSTALL(TARGETS hello hello_static
#        LIBRARY DESTINATION include/lib
#        ARCHIVE DESTINATION include/lib)

INSTALL(DIRECTORY algorithm DESTINATION include/LA-MAPF)


#add_library(external_lib ${third1_src} ${third2_src} ${third3_src})



#add_executable(test_lacam3 test/test_lacam3.cpp)
#target_link_libraries(test_lacam3 lamapf_alg)

#add_executable(test_massive_decomposition test/test_massive_decomposition.cpp)
#target_link_libraries(test_massive_decomposition lamapf_alg)

#add_executable(test_massive_layered_mapf test/test_massive_layered_mapf.cpp)
#target_link_libraries(test_massive_layered_mapf lamapf_alg)


add_executable(test_lamapf_instance_generator test/large_agent_test/test_lamapf_instance_generator.cpp)
target_link_libraries(test_lamapf_instance_generator lamapf_alg)

#add_executable(test_large_agent_mapf_decomposition test/large_agent_test/test_large_agent_mapf_decomposition.cpp)
#target_link_libraries(test_large_agent_mapf_decomposition lamapf_alg)

#add_executable(test_layered_large_agent_mapf test/large_agent_test/test_layered_large_agent_mapf.cpp)
#target_link_libraries(test_layered_large_agent_mapf lamapf_alg)


#add_executable(test_hybird_mapf third_party/Hybrid_MAPF/main.cpp)
#target_link_libraries(test_hybird_mapf freeNav-base lamapf_alg)