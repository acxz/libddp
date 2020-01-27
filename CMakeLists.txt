cmake_minimum_required(VERSION 3.3)

project(libddp VERSION 0.0.1 LANGUAGES CXX)

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Build
## add include directory
include_directories(include)
## add include directory for eigen
include_directories(${Eigen_INCLUDE_DIRS})

## build examples
add_subdirectory(examples)

# Install
## for correct library locations across platforms
include(GNUInstallDirs)

## install the header files
install(DIRECTORY include/libddp DESTINATION include)

## copy the cmake files required for find_package()
# TODO: confirm this works
install(FILES cmake/libddpConfig.cmake DESTINATION share/libddp/cmake)

# Uninstall
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
