cmake_minimum_required(VERSION 3.15)
project(uxsim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python and nanobind for bindings
find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(nanobind CONFIG REQUIRED)

# Python extension module
# Note: bindings.cpp includes traffi.cpp directly via #include "traffi.cpp",
# so we only compile bindings.cpp as the single translation unit.
nanobind_add_module(uxsim_cpp
    bindings.cpp
)

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
