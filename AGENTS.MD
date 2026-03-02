# AGENTS.MD

## Purpose
Build and run the Direct3D 12 flight prototype in this repository using MSVC + CMake + vcpkg manifest mode.

## Project assumptions
- OS: Windows 10/11 x64
- Compiler: MSVC (Visual Studio 2022)
- Build system: CMake
- Package manager: vcpkg (manifest mode via `vcpkg.json`)
- Graphics API: Direct3D 12 (Win32 window)

## Important paths
- Source root: `C:\Users\Jacob\Documents\code\3DFlight`
- Assets used at runtime: `assets/`
- Shaders used at runtime: `shaders/`
- Built exe (Debug): `build\Debug\d3d12_flight.exe`

## One-time prerequisites
1. Install Visual Studio 2022 with Desktop C++ workload.
2. Install vcpkg and bootstrap it.
3. Ensure this file exists:
   - `C:\vcpkg\scripts\buildsystems\vcpkg.cmake`

## Configure + Build (recommended from WSL/bash)
Use explicit Windows CMake path to avoid Linux generator issues.

```bash
"/mnt/c/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" \
  -S . -B build -G "Visual Studio 17 2022" -A x64 \
  -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"

"/mnt/c/Program Files/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" \
  --build build --config Debug
```

## Configure + Build (Developer Command Prompt / PowerShell)
```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Debug
```

## Run
Run from the output directory so copied `assets/` and `shaders/` are found.

```bat
cd /d C:\Users\Jacob\Documents\code\3DFlight\build\Debug
d3d12_flight.exe
```

## Clean rebuild
```bat
rmdir /s /q build
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Debug
```

## Common failures and fixes
- Error: `Could not create named generator Visual Studio 17 2022`
  - Cause: Linux CMake was used.
  - Fix: Use the explicit Windows `cmake.exe` path shown above.

- Error: `cmake is not recognized` in `cmd.exe`
  - Cause: CMake not on that shell PATH.
  - Fix: Use Developer Command Prompt or explicit full path to `cmake.exe`.

- Missing ImGui backend headers
  - Use `#include <imgui_impl_win32.h>` and `#include <imgui_impl_dx12.h>` (not `backends/...`).

## Notes for agents
- Do not delete or move existing root assets (`Airplane.glb`, Natural Earth folders, landmask files).
- If needed, copy assets into `assets/` rather than moving originals.
- Keep runtime dependencies as Win32 + D3D12 + ImGui (no SFML in this prototype).
