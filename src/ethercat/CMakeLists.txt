cmake_minimum_required(VERSION 3.16)
project(multi_robot_ethercat LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------- Required dependencies ----------
find_package(PkgConfig REQUIRED)
pkg_check_modules(ETHERCAT REQUIRED IMPORTED_TARGET libethercat)

find_package(Threads REQUIRED)

# POSIX real-time library
find_library(RT_LIBRARY rt REQUIRED)

# ---------- Core library ----------
add_library(ethercat_layer STATIC
    EthercatMaster.cpp
    RobotSlaveDriver.cpp
    TurntableController.cpp
    ScrewDriverController.cpp
    SafetyMonitor.cpp
)

target_include_directories(ethercat_layer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ETHERCAT_INCLUDE_DIRS}
)

target_link_libraries(ethercat_layer PUBLIC
    PkgConfig::ETHERCAT
    Threads::Threads
    ${RT_LIBRARY}
)

target_compile_options(ethercat_layer PRIVATE
    -Wall -Wextra -Wpedantic -Wno-unused-parameter
    -O2 -fno-exceptions        # real-time: avoid exception overhead in hot path
)

# ---------- Optional: Python bindings ----------
option(BUILD_PYTHON_BINDINGS "Build pybind11 Python bindings" OFF)

if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 CONFIG REQUIRED)
    pybind11_add_module(ethercat_py pybind_module.cpp)
    target_link_libraries(ethercat_py PRIVATE ethercat_layer)
endif()
