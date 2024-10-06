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


#########################################################
# FIND Pangolin
#########################################################
find_package(Pangolin REQUIRED)


#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
find_package(Octomap REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

add_subdirectory(third_party/grid-pathfinding/graph ./graph)


add_subdirectory(freeNav-base)
include_directories(third_party)


# 开启QT用于预处理的组件
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 查找QT的模块
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Charts REQUIRED)

set(QT_LIBS
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::PrintSupport
        Qt5::Charts
        )


INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Gui_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5OpenGL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5PrintSupport_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Charts_INCLUDE_DIRS})

aux_source_directory(test/visualization/qt_gui qt_src)

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

add_library(EECBS_lib  ${EECBS_src} ${PBS_src} ${lacam_src} ${CBSH2-RTC_src} ${lacam2_src} ${lacam3_src}
        ${MAPF_LNS2_src} ${MAPF_LNS2_CBS_src} ${MAPF_LNS2_PIBT_src} ${PIBT2_src})

aux_source_directory(test/visualization vis_src)
add_library(vis_lib ${vis_src} ${qt_src})
target_link_libraries(vis_lib ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${Pangolin_LIBRARIES} ${QT_LIBS})

aux_source_directory(algorithm alg_src)
aux_source_directory(algorithm/LA-MAPF lamapf_src)
add_library(alg_lib ${alg_src} ${lamapf_src})
target_link_libraries(alg_lib EECBS_lib vis_lib lib-graph)
#add_library(external_lib ${third1_src} ${third2_src} ${third3_src})

add_executable(test_2d_mapf_viewer test/test_2d_mapf_viewer.cpp)
target_link_libraries(test_2d_mapf_viewer freeNav-base alg_lib)

add_executable(test_lacam test/test_lacam.cpp)
target_link_libraries(test_lacam freeNav-base alg_lib)

add_executable(test_lacam3 test/test_lacam3.cpp)
target_link_libraries(test_lacam3 freeNav-base alg_lib)

add_executable(test_massive_decomposition test/test_massive_decomposition.cpp)
target_link_libraries(test_massive_decomposition freeNav-base alg_lib)

add_executable(test_massive_layered_mapf test/test_massive_layered_mapf.cpp)
target_link_libraries(test_massive_layered_mapf freeNav-base alg_lib)

add_executable(test_create_random_map_fr_mapf test/test_create_random_map_fr_mapf.cpp)
target_link_libraries(test_create_random_map_fr_mapf freeNav-base alg_lib)

add_executable(test_qt_gui test/test_qt_gui.cpp test/visualization/qt_gui/resources/images.qrc)
target_link_libraries(test_qt_gui freeNav-base alg_lib)

add_executable(test_lamapf_cbs test/large_agent_test/test_lamapf_cbs.cpp)
target_link_libraries(test_lamapf_cbs freeNav-base alg_lib)

add_executable(test_lamapf_lacam test/large_agent_test/test_lamapf_lacam.cpp)
target_link_libraries(test_lamapf_lacam freeNav-base alg_lib)

add_executable(test_lamapf_instance_generator test/large_agent_test/test_lamapf_instance_generator.cpp)
target_link_libraries(test_lamapf_instance_generator freeNav-base alg_lib)

add_executable(test_coverage_grid test/large_agent_test/test_coverage_grid.cpp)
target_link_libraries(test_coverage_grid freeNav-base alg_lib)

add_executable(test_large_agent_mapf_decomposition test/large_agent_test/test_large_agent_mapf_decomposition.cpp)
target_link_libraries(test_large_agent_mapf_decomposition freeNav-base alg_lib)

add_executable(test_layered_large_agent_mapf test/large_agent_test/test_layered_large_agent_mapf.cpp)
target_link_libraries(test_layered_large_agent_mapf freeNav-base alg_lib)
