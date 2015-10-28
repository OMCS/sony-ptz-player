# CMake configuration for building the sony-ptz driver, requires CMake >= 2.7
project (Sony-PTZ) 
cmake_minimum_required (VERSION 2.7 FATAL_ERROR)
#SET (CMAKE_MODULE_PATH /${prefix}/share/cmake/Modules)
SET (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
SET (CMAKE_SHARED_LIBRARY_PREFIX "") # Do not build driver with the standard 'lib' prefix

SET(CMAKE_BUILD_TYPE Debug) # Include symbol table for debugging, remove for production driver

INCLUDE (UsePlayerPlugin)

PLAYER_ADD_PLUGIN_DRIVER (sony-ptz SOURCES sony-ptz.cc)

