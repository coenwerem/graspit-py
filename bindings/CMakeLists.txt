cmake_minimum_required(VERSION 3.16)

project(graspit_bindings VERSION 0.1.0 LANGUAGES CXX)

# CMake 3.28+ removes FindPythonInterp/FindPythonLibs; ensure dependencies use
# modern FindPython to avoid CMP0148 warnings.
if(POLICY CMP0148)
    cmake_policy(SET CMP0148 NEW)
endif()

# Let pybind11 prefer modern FindPython.
set(PYBIND11_FINDPYTHON ON)

# Find required packages
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(PkgConfig REQUIRED)

# GraspIt headers depend on Qt.
find_package(Qt5 COMPONENTS Core Widgets Network Sql REQUIRED)

# Find GraspIt!.
# Preferred: an installed GraspIt package config (via `sudo make install`).
# Fallback: link directly against the locally-built libgraspit.so from this mono-repo.

set(GRASPIT_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../graspit" CACHE PATH "Path to GraspIt source directory")
set(GRASPIT_BUILD_DIR "${GRASPIT_SOURCE_DIR}/build" CACHE PATH "Path to GraspIt build directory")

# Ensure GraspIt Qt resources (icons/images) are available when using GraspIt as a library.
# The standalone graspit_simulator executable embeds these resources, but libgraspit.so does not.
qt5_add_resources(_graspit_qrc_sources "${GRASPIT_SOURCE_DIR}/src/graspitResources.qrc")

# Only hint CMake with a GraspIt prefix if it provides exported targets.
if(EXISTS "${GRASPIT_BUILD_DIR}/graspitConfig.cmake" AND EXISTS "${GRASPIT_BUILD_DIR}/graspitTargets.cmake")
    list(APPEND CMAKE_PREFIX_PATH "${GRASPIT_BUILD_DIR}")
endif()

find_package(graspit CONFIG QUIET)

set(_use_graspit_config OFF)
if(graspit_FOUND)
    # Some build trees may have graspitConfig.cmake but no exported targets; guard against that.
    if(DEFINED graspit_DIR AND EXISTS "${graspit_DIR}/graspitTargets.cmake")
        set(_use_graspit_config ON)
    endif()
endif()

if(_use_graspit_config)
    set(_graspit_link_libs ${GRASPIT_LIBRARIES})
    set(_graspit_include_dirs ${GRASPIT_INCLUDE_DIRS})
else()
    # Local-build fallback (no install / no exported targets).
    if(NOT EXISTS "${GRASPIT_BUILD_DIR}/libgraspit.so")
        message(FATAL_ERROR "GraspIt not found. Either install GraspIt (sudo make install) so CMake can find graspitConfig.cmake, or set GRASPIT_BUILD_DIR to a build dir containing libgraspit.so")
    endif()

    set(_graspit_link_libs "${GRASPIT_BUILD_DIR}/libgraspit.so")
    set(_graspit_include_dirs
        "${GRASPIT_SOURCE_DIR}/include"
        "${GRASPIT_SOURCE_DIR}/src"
        "${GRASPIT_SOURCE_DIR}/ui"
        "${GRASPIT_BUILD_DIR}/include"
        "${GRASPIT_BUILD_DIR}/include/graspit/ui"
    )
endif()

if(EXISTS "${GRASPIT_SOURCE_DIR}/tinyxml")
    list(APPEND _graspit_include_dirs "${GRASPIT_SOURCE_DIR}/tinyxml")
endif()

# pybind11 module
pybind11_add_module(_bindings graspit_bindings.cpp ${_graspit_qrc_sources})

target_include_directories(_bindings PRIVATE
    ${_graspit_include_dirs}
)

# Link with GraspIt! libraries
target_link_libraries(_bindings PRIVATE
    ${_graspit_link_libs}
    Qt5::Core
    Qt5::Widgets
    Qt5::Network
    Qt5::Sql
)

# Set C++ standard
target_compile_features(_bindings PRIVATE cxx_std_17)

# Compiler definitions
target_compile_definitions(_bindings PRIVATE VERSION_INFO="${PROJECT_VERSION}")

# Help the dynamic loader find libgraspit.so when using the local-build fallback.
if(EXISTS "${GRASPIT_BUILD_DIR}/libgraspit.so")
    set_target_properties(_bindings PROPERTIES
        BUILD_RPATH "${GRASPIT_BUILD_DIR}"
        INSTALL_RPATH "${GRASPIT_BUILD_DIR}"
    )
endif()