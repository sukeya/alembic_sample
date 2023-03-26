cmake_minimum_required(VERSION 3.25.2)


project(abcprint VERSION 0.1.0 LANGUAGES CXX)


# In-source build guard
if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()


# Config
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -march=native -mtune=native")
set(CMAKE_OPTIMIZE_DEPENDENCIES ON)


# Library
# argparse
find_path(ARGPARSE_INCLUDE_DIR NAMES argparse.hpp PATH_SUFFIXES argparse)
include_directories(${ARGPARSE_INCLUDE_DIR})

# Alembic
find_package(Alembic CONFIG REQUIRED)
list(APPEND EXTRA_LIBS Alembic::Alembic)

# Boost
set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
set(Boost_USE_DEBUG_LIBS        OFF)  # ignore debug libs and
set(Boost_USE_RELEASE_LIBS       ON)  # only find release libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
list(APPEND EXTRA_LIBS Boost::unit_test_framework)
include_directories(${Boost_INCLUDE_DIRS})

# fmt
find_package(fmt CONFIG REQUIRED)
list(APPEND EXTRA_LIBS fmt::fmt)

# nlohmann json
set(nlohmann-json_IMPLICIT_CONVERSIONS OFF)
find_path(NLOHMANN_INCLUDE_DIR NAMES json.hpp PATH_SUFFIXES nlohmann)
include_directories(${NLOHMANN_INCLUDE_DIR})


# Build target
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${EXTRA_LIBS})
