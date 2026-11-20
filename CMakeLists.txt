cmake_minimum_required(VERSION 3.16)
project(robot_ga VERSION 0.1.0 LANGUAGES CXX)

option(ROBOT_GA_BUILD_EXE    "Program robot_ga (src/main.cpp)"   ON)
option(ROBOT_GA_BUILD_PYTHON "Modul Pythona robot_ga (pybind11)" ON)
option(ROBOT_GA_BUILD_TESTS  "Unit tests (GoogleTest)"           OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --- Biblioteka z algorytmem: wszystko z src/ poza main.cpp ---
file(GLOB CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_library(robot_ga_core STATIC ${CORE_SOURCES})
target_include_directories(robot_ga_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
if(MSVC)
    target_compile_options(robot_ga_core PRIVATE /W4)
else()
    target_compile_options(robot_ga_core PRIVATE -Wall -Wextra)
endif()
set_target_properties(robot_ga_core PROPERTIES POSITION_INDEPENDENT_CODE ON)   # trafia do .so

# --- Program terminalowy ---
if(ROBOT_GA_BUILD_EXE)
    add_executable(robot_ga src/main.cpp)
    target_link_libraries(robot_ga PRIVATE robot_ga_core)
    if(MSVC)
        target_compile_options(robot_ga PRIVATE /W4)
    else()
        target_compile_options(robot_ga PRIVATE -Wall -Wextra)
    endif()
endif()

# --- Modul Pythona: import robot_ga ---
if(ROBOT_GA_BUILD_PYTHON)
    find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)
    find_package(pybind11 CONFIG REQUIRED)

    pybind11_add_module(_core bindings/module.cpp)
    target_link_libraries(_core PRIVATE robot_ga_core)

    if(SKBUILD)
        # pip install .
        install(TARGETS _core LIBRARY DESTINATION robot_ga)
    else()
        # zwykle budowanie: modul laduje obok pakietu (PYTHONPATH=python)
        set_target_properties(_core PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/robot_ga
            # generatory wieloconfigowe (Visual Studio) dokladaja podkatalog Release/Debug
            LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/python/robot_ga
            LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/python/robot_ga
            LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_CURRENT_SOURCE_DIR}/python/robot_ga
            LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL ${CMAKE_CURRENT_SOURCE_DIR}/python/robot_ga)
    endif()
endif()

# --- Unit tests: cmake -DROBOT_GA_BUILD_TESTS=ON, then ctest ---
if(ROBOT_GA_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
        GIT_SHALLOW TRUE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(robot_ga_tests tests/test_core.cpp)
    target_link_libraries(robot_ga_tests PRIVATE robot_ga_core GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(robot_ga_tests)
endif()
