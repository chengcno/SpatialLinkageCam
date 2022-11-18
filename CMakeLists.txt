cmake_minimum_required(VERSION 3.1)
project(SpatialLinkages)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

#########################################
#####                               #####
#####           Option              #####
#####                               #####
#########################################

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_CGAL              "Use CGAL"           ON)

#########################################
#####                               #####
#####      External Library         #####
#####                               #####
#########################################

#EIGEN
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#Boost
find_package(Boost REQUIRED COMPONENTS system filesystem thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#3rd party library
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/ext/")

#########################################
#####                               #####
#####      Core Library             #####
#####                               #####
#########################################
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

file(GLOB 3DCoreFiles
        src/Mesh/*.h
        src/Mesh/*.cpp
        src/Utility/*.h
        src/Utility/*.cpp)

include_directories(src)

add_library(3DCorelib STATIC ${3DCoreFiles})
target_compile_options(3DCorelib PUBLIC "-Wno-deprecated-declarations")

target_include_directories(3DCorelib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/
        )

target_link_libraries(3DCorelib PUBLIC
        ${ext_lib})

target_link_libraries(3DCorelib PUBLIC ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY})
#########################################
#####                               #####
#####      Gui Library              #####
#####                               #####
#########################################

file(GLOB 3DRenderFiles
        src/Render/*.h
        src/Render/*.cpp)

add_library(3DRenderlib STATIC ${3DRenderFiles})

target_include_directories(3DRenderlib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Render/
        )

target_link_libraries(3DRenderlib PUBLIC
        ${ext_lib})

#########################################
#####                               #####
#####       Linkages & Cam          #####
#####                               #####
#########################################

file(GLOB LinkagesFiles
        src/Linkages/*.h
        src/Linkages/*.cpp
        src/CamPart/*.h
        src/CamPart/*.cpp
        src/CamLinks/*.h
        src/CamLinks/*.cpp)

add_library(Linkageslib STATIC ${LinkagesFiles})

target_include_directories(Linkageslib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Linkages/
        )

target_link_libraries(Linkageslib PUBLIC
        ${ext_lib})

#########################################
#####                               #####
#####           Optimizer           #####
#####                               #####
#########################################

file(GLOB OptFiles
        src/Opt/*.h
        src/Opt/*.cpp
        src/IOfile/*.h
        src/IOfile/*.cpp)

add_library(Optlib STATIC ${OptFiles})

target_include_directories(Optlib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Opt/
        )

target_link_libraries(Optlib PUBLIC ${ext_lib})


#########################################
#####                               #####
#####         release Main          #####
#####                               #####
#########################################

add_executable(${PROJECT_NAME}_rMain src/MainFunc/releaseMain.cpp)
target_link_libraries(${PROJECT_NAME}_rMain PUBLIC 3DCorelib)
target_link_libraries(${PROJECT_NAME}_rMain PUBLIC 3DRenderlib)
target_link_libraries(${PROJECT_NAME}_rMain PUBLIC Linkageslib)
target_link_libraries(${PROJECT_NAME}_rMain PUBLIC Optlib)


