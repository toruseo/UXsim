cmake_minimum_required(VERSION 3.15)
project(uxsim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find pybind11 for Python bindings
find_package(pybind11 REQUIRED CONFIG)

# Python extension module
# Note: bindings.cpp includes traffi.cpp directly via #include "traffi.cpp",
# so we only compile bindings.cpp as the single translation unit.
pybind11_add_module(uxsim_cpp
    bindings.cpp
)

# --- OpenMP (optional) ---
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(uxsim_cpp PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(uxsim_cpp PRIVATE UXSIM_USE_OPENMP)
    message(STATUS "OpenMP found - parallel execution enabled")
else()
    message(STATUS "OpenMP not found - building single-threaded")
endif()

# Install into the uxsim package directory
install(TARGETS uxsim_cpp
        LIBRARY DESTINATION uxsim
        COMPONENT python)

# Optimization flags
if(MSVC)
    target_compile_options(uxsim_cpp PRIVATE /O2)
else()
    target_compile_options(uxsim_cpp PRIVATE -O3)
endif()

# Static linking for MinGW
if(MINGW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++ -static -O3")
endif()
