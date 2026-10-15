cmake_minimum_required(VERSION 3.20)
project(aegislink LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# scikit-build-core sets SKBUILD when pip builds the Python module; the C++ tests are not needed then.
if(SKBUILD)
  set(AEGIS_TESTS_DEFAULT OFF)
else()
  set(AEGIS_TESTS_DEFAULT ON)
endif()
option(AEGIS_BUILD_TESTS "Build the C++ unit tests" ${AEGIS_TESTS_DEFAULT})
option(AEGIS_SANITIZE "Build with AddressSanitizer and UBSan (GCC/Clang)" OFF)

if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

if(AEGIS_SANITIZE AND NOT MSVC)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_subdirectory(cpp/core)

if(SKBUILD)
  # Found first so pybind11 uses CMake's FindPython instead of its deprecated fallback.
  find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(_core cpp/bindings/aegis_core.cpp)
  target_link_libraries(_core PRIVATE aegis_core)
  install(TARGETS _core DESTINATION aegislink)
endif()

if(AEGIS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(cpp/tests)
endif()
