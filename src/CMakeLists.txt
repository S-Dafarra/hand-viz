#===============================================================================
#
# Copyright (C) 2022Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME icub-hand-viz)

option(USE_SYSTEM_RobotsIO OFF)
if(USE_SYSTEM_RobotsIO)
   find_package(RobotsIO REQUIRED)
else()
  include(FetchContent)
  FetchContent_Declare(RobotsIO
    GIT_REPOSITORY https://github.com/S-Dafarra/robots-io
    GIT_TAG 2dfe590e3b0501387ed8dedc31b4339eb36474a7)

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
    GIT_TAG 4c3c472fd69b9c066feaea032d9f15f867db2b93)

  FetchContent_GetProperties(RobotsViz)
  if(NOT RobotsViz_POPULATED)
    message(STATUS "Fetching RobotsViz...")
    FetchContent_MakeAvailable(RobotsViz)
  endif()
endif()


find_package(RobotsViz REQUIRED)

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
