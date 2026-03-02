cmake_minimum_required(VERSION 3.24)

project(D3D12FlightPrototype LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(imgui CONFIG REQUIRED)
find_package(unofficial-gdal CONFIG QUIET)
find_package(GDAL CONFIG QUIET)

add_executable(d3d12_flight WIN32
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

target_include_directories(d3d12_flight PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cgltf
)

target_compile_definitions(d3d12_flight PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    UNICODE
    _UNICODE
)

if(MSVC)
    target_compile_options(d3d12_flight PRIVATE /W4 /permissive-)
endif()

target_link_libraries(d3d12_flight PRIVATE
    imgui::imgui
    d3d12
    dxgi
    dxguid
    d3dcompiler
    windowscodecs
)

if(TARGET unofficial::gdal::gdal)
    target_link_libraries(d3d12_flight PRIVATE unofficial::gdal::gdal)
elseif(TARGET GDAL::GDAL)
    target_link_libraries(d3d12_flight PRIVATE GDAL::GDAL)
elseif(TARGET gdal)
    target_link_libraries(d3d12_flight PRIVATE gdal)
else()
    message(FATAL_ERROR "GDAL target not found. Ensure vcpkg gdal package is installed.")
endif()

set_target_properties(d3d12_flight PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:d3d12_flight>"
)

add_custom_command(TARGET d3d12_flight POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/shaders
            $<TARGET_FILE_DIR:d3d12_flight>/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:d3d12_flight>/assets
)
