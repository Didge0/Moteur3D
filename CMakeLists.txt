cmake_minimum_required(VERSION 3.20)
project(Moteur3D VERSION 1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Matrice
)

# ================= Moteur Library =================
file(GLOB_RECURSE SRCS_MOTEUR CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Moteur3D/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Moteur3D/*.h
)
add_library(Moteur STATIC ${SRCS_MOTEUR})
target_include_directories(Moteur PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Moteur3D
)
target_link_libraries(Moteur
    PUBLIC Matrice
    PUBLIC SDL3::SDL3
)


# ================= Executable =================
add_executable(${PROJECT_NAME} "src/main.c")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PRIVATE Moteur)
