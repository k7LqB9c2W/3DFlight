cmake_minimum_required(VERSION 3.24)

project(3DFlight LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(imgui CONFIG REQUIRED)
find_package(unofficial-gdal CONFIG QUIET)
find_package(GDAL CONFIG QUIET)

add_executable(3DFlight WIN32
    src/main.cpp
    src/d3d12_renderer.cpp
    src/d3d12_renderer.h
    src/mesh.cpp
    src/mesh.h
    src/gltf_loader.cpp
    src/gltf_loader.h
    src/earth.cpp
    src/earth.h
    src/sim.cpp
    src/sim.h
    src/terrain/etopo_tile.cpp
    src/terrain/etopo_tile.h
    src/terrain/terrain_system.cpp
    src/terrain/terrain_system.h
    src/terrain/terrain_mesh.cpp
    src/terrain/terrain_mesh.h
    third_party/cgltf/cgltf.c
)

set_source_files_properties(third_party/cgltf/cgltf.c PROPERTIES LANGUAGE C)

target_include_directories(3DFlight PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cgltf
)

target_compile_definitions(3DFlight PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    UNICODE
    _UNICODE
)

if(MSVC)
    target_compile_options(3DFlight PRIVATE /W4 /permissive-)
endif()

target_link_libraries(3DFlight PRIVATE
    imgui::imgui
    d3d12
    dxgi
    dxguid
    d3dcompiler
    windowscodecs
)

if(TARGET unofficial::gdal::gdal)
    target_link_libraries(3DFlight PRIVATE unofficial::gdal::gdal)
elseif(TARGET GDAL::GDAL)
    target_link_libraries(3DFlight PRIVATE GDAL::GDAL)
elseif(TARGET gdal)
    target_link_libraries(3DFlight PRIVATE gdal)
else()
    message(FATAL_ERROR "GDAL target not found. Ensure vcpkg gdal package is installed.")
endif()

set_target_properties(3DFlight PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:3DFlight>"
)

add_custom_command(TARGET 3DFlight POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/shaders
            $<TARGET_FILE_DIR:3DFlight>/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:3DFlight>/assets
)
