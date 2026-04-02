cmake_minimum_required(VERSION 3.20)
project(Moteur3D VERSION 1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(MOTEUR3D_BUILD_EXAMPLES "Build Moteur3D example executable" ON)

# ================= SDL3 =================
set(SDL3_DIR "C:/Library/SDL3")
find_package(SDL3 REQUIRED CONFIG)

# ================= Matrice Library =================
file(GLOB_RECURSE SRCS_MATRICE CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrice/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrice/*.h
)
add_library(Matrice STATIC ${SRCS_MATRICE})
target_include_directories(Matrice PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Matrice>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Matrice>
)

# ================= Moteur Library =================
file(GLOB_RECURSE SRCS_MOTEUR CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Moteur3D/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Moteur3D/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Moteur3D/*.h
)
add_library(Moteur STATIC ${SRCS_MOTEUR})
target_include_directories(Moteur PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_include_directories(Moteur PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Moteur3D
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrice
)
target_link_libraries(Moteur
    PUBLIC Matrice
    PUBLIC SDL3::SDL3
)

# Enforce explicit function declarations in consumers too.
target_compile_options(Moteur INTERFACE
    $<$<C_COMPILER_ID:MSVC>:/we4013>
    $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Werror=implicit-function-declaration>
)

# ================= Executable =================
if(MOTEUR3D_BUILD_EXAMPLES)
    add_executable(${PROJECT_NAME} "examples/basic/main.c")
    target_link_libraries(${PROJECT_NAME} PRIVATE Moteur)
endif()

# ================= Install / Package Export =================
install(TARGETS Matrice Moteur
    EXPORT Moteur3DTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrice/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Matrice
    FILES_MATCHING PATTERN "*.h"
)

install(EXPORT Moteur3DTargets
    FILE Moteur3DTargets.cmake
    NAMESPACE Moteur3D::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Moteur3D
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Moteur3DConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/Moteur3DConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Moteur3D
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/Moteur3DConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/Moteur3DConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/Moteur3DConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Moteur3D
)
