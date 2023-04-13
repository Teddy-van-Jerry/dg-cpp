cmake_minimum_required(VERSION 3.0.0)
project(dg-cpp VERSION 0.1.0 LANGUAGES CXX)

option(DG_CPP_TESTS "Run dg-cpp tests" ON)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

list(APPEND DG_CPP_FILES
    "include/dg.hpp"
    "include/dg-cpp/base.hpp"
    "include/dg-cpp/common.hpp"
    "include/dg-cpp/core.hpp"
    "include/dg-cpp/edge.hpp"
    "include/dg-cpp/except.hpp"
    "include/dg-cpp/node.hpp"
)

add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_include_directories(${CMAKE_PROJECT_NAME} INTERFACE include)
target_sources(
    ${CMAKE_PROJECT_NAME}
    INTERFACE FILE_SET HEADERS
    BASE_DIRS ${CMAKE_SOURCE_DIR}/include
    FILES ${DG_CPP_FILES}
)

install(
    TARGETS ${CMAKE_PROJECT_NAME}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if (DG_CPP_TESTS)
    message("Build DG CPP Tests")
    add_subdirectory(${CMAKE_SOURCE_DIR}/test ${CMAKE_SOURCE_DIR}/bin)
endif()
