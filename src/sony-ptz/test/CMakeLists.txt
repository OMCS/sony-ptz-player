# CMake configuration for building the sony-ptz driver, requires CMake >= 2.7
project (Sony-PTZ-Test) 
cmake_minimum_required (VERSION 2.7 FATAL_ERROR)
#SET (CMAKE_MODULE_PATH /${prefix}/share/cmake/Modules)
SET (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules)

SET(CMAKE_BUILD_TYPE Debug) # Include symbol table for debugging, remove for production driver

INCLUDE (UsePlayerC++)

PLAYER_ADD_PLAYERCPP_CLIENT (sony-ptz-test SOURCES test.cc)

