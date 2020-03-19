cmake_minimum_required(VERSION 2.8)
project(ExplorationORB)

INCLUDE("Setup.cmake")

Setup()

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

## Specify additional locations of header files
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/src_orbslam
    ${CMAKE_CURRENT_LIST_DIR}/src/src_exploration
    ${CMAKE_CURRENT_LIST_DIR}/src/src_robot
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/src/src_airsim
)

file(GLOB SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/src_orbslam/*.cc
    ${CMAKE_CURRENT_LIST_DIR}/src/src_exploration/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/src_robot/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/src_airsim/*.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${FREEGLUT_LIBRARY}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

if(AIRSIM_ROOT)
    target_link_libraries(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/Thirdparty//LibsAirSim/libAirLib.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/LibsAirSim/librpc.a
    )
endif()

# Build exploration_airsim
if(AIRSIM_ROOT)
    add_executable(exploration_airsim src/main_airsim.cpp)
    target_link_libraries(exploration_airsim ${PROJECT_NAME})
endif(AIRSIM_ROOT)
