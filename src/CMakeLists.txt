#===============================================================================
#
# Copyright (C) 2022Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME icub-hand-viz)

# Force the use of YARP and ICUB
set(USE_YARP ON CACHE INTERNAL "Use YARP")
set(USE_ICUB ON CACHE INTERNAL "Use ICUB")

option(USE_SYSTEM_RobotsIO OFF)
if(USE_SYSTEM_RobotsIO)
find_package(RobotsIO REQUIRED)
else()
  include(FetchContent)
  FetchContent_Declare(RobotsIO
    GIT_REPOSITORY https://github.com/S-Dafarra/robots-io
    GIT_TAG ed0481a49143f83d599433d1310c00c80e75bad3)

  FetchContent_GetProperties(RobotsIO)
  if(NOT RobotsIO_POPULATED)
    message(STATUS "Fetching RobotsIO...")
    FetchContent_MakeAvailable(RobotsIO)
  endif()
endif()

option(USE_SYSTEM_RobotsViz OFF)
if(USE_SYSTEM_RobotsViz)
   find_package(RobotsViz REQUIRED)
else()
  include(FetchContent)
  FetchContent_Declare(RobotsViz
    GIT_REPOSITORY https://github.com/S-Dafarra/robots-viz
    GIT_TAG f6a6cb10a70546a864e0ae74ec72578478b82200)

  FetchContent_GetProperties(RobotsViz)
  if(NOT RobotsViz_POPULATED)
    message(STATUS "Fetching RobotsViz...")
    FetchContent_MakeAvailable(RobotsViz)
  endif()
endif()

find_package(YARP REQUIRED COMPONENTS
             cv    # seems that RobotsIO export is not working on Ubuntu18.04
             eigen # seems that RobotsIO export is not working on Ubuntu18.04
             idl_tools
             os
             dev
)

set(${EXE_TARGET_NAME}_HDR
    include/Eye.h
    include/HandsVisualizer.h)

set(${EXE_TARGET_NAME}_SRC
    src/Eye.cpp
    src/HandsVisualizer.cpp
    src/main.cpp
)

set (THRIFTS thrifts/HandVisualizerCommands.thrift)

yarp_add_idl(THRIFT_GEN_FILES ${THRIFTS})

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC} ${THRIFT_GEN_FILES})

target_include_directories(${EXE_TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME}
                      YARP::YARP_os
                      YARP::YARP_dev
                      RobotsIO::RobotsIO
                      RobotsViz::RobotsViz
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
