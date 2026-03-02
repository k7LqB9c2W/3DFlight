#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>

#include <imgui.h>
#include <imgui_impl_win32.h>

#include "d3d12_renderer.h"
#include "gltf_loader.h"
#include "mesh.h"
#include "sim.h"
#include "terrain/terrain_mesh.h"
#include "terrain/terrain_system.h"

using flight::D3D12Renderer;
using flight::FlightSim;
using flight::FlightStart;
using flight::InputState;
using flight::MeshData;
using flight::terrain::TerrainPatchBuildResult;
using flight::terrain::TerrainPatchSettings;
using flight::terrain::TerrainSystem;
using flight::terrain::TileKey;

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam);

namespace {

D3D12Renderer* g_renderer = nullptr;

LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam) {
    if (ImGui_ImplWin32_WndProcHandler(hwnd, msg, wparam, lparam)) {
        return 1;
    }

    switch (msg) {
        case WM_SIZE:
            if (g_renderer != nullptr && wparam != SIZE_MINIMIZED) {
                const uint32_t w = static_cast<uint32_t>(LOWORD(lparam));
                const uint32_t h = static_cast<uint32_t>(HIWORD(lparam));
                g_renderer->Resize(w, h);
            }
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProc(hwnd, msg, wparam, lparam);
    }
}

bool IsKeyDown(int vk) {
    return (GetAsyncKeyState(vk) & 0x8000) != 0;
}

double LonDeltaDeg(double aDeg, double bDeg) {
    double d = aDeg - bDeg;
    while (d > 180.0) {
        d -= 360.0;
    }
    while (d < -180.0) {
        d += 360.0;
    }
    return d;
}

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int showCmd) {
    CoInitializeEx(nullptr, COINITBASE_MULTITHREADED);

    const wchar_t* kClassName = L"D3D12FlightPrototypeClass";

    WNDCLASSW wc{};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = instance;
    wc.lpszClassName = kClassName;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);

    if (!RegisterClassW(&wc)) {
        MessageBoxA(nullptr, "RegisterClassW failed", "Error", MB_OK | MB_ICONERROR);
        return 1;
    }

    RECT rect{0, 0, 1600, 900};
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);

    HWND hwnd = CreateWindowExW(
        0,
        kClassName,
        L"D3D12 Flight Prototype",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        rect.right - rect.left,
        rect.bottom - rect.top,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (!hwnd) {
        MessageBoxA(nullptr, "CreateWindowExW failed", "Error", MB_OK | MB_ICONERROR);
        return 1;
    }

    ShowWindow(hwnd, showCmd);

    D3D12Renderer renderer;
    g_renderer = &renderer;

    std::string initError;
    if (!renderer.Initialize(hwnd, 1600, 900, L"shaders", L"assets", initError)) {
        MessageBoxA(hwnd, initError.c_str(), "Renderer Init Error", MB_OK | MB_ICONERROR);
        g_renderer = nullptr;
        return 1;
    }

    MeshData planeMesh;
    std::string meshError;
    const std::filesystem::path modelPath = std::filesystem::path("assets") / "models" / "Airplane.glb";
    const bool glbLoaded = flight::LoadGlbMesh(modelPath, planeMesh, meshError);
    if (!glbLoaded) {
        planeMesh = flight::CreatePlaceholderPlane();
    } else {
        // Align this GLB's local facing with the simulation forward direction.
        for (auto& v : planeMesh.vertices) {
            v.position.x = -v.position.x;
            v.position.z = -v.position.z;
            v.normal.x = -v.normal.x;
            v.normal.z = -v.normal.z;
        }
    }

    std::string uploadError;
    if (!renderer.SetPlaneMesh(planeMesh, uploadError)) {
        MessageBoxA(hwnd, uploadError.c_str(), "Plane Upload Error", MB_OK | MB_ICONERROR);
    }

    FlightSim sim;
    FlightStart editableStart = sim.GetStart();
    float timeScale = 1.0f;

    TerrainSystem terrainSystem;
    bool terrainSystemReady = false;
    std::string terrainInitError;
    if (!terrainSystem.Initialize(std::filesystem::path("assets") / "etopo" / "tiles", 9, terrainInitError)) {
        terrainInitError = "Terrain init failed: " + terrainInitError;
    } else {
        terrainSystemReady = true;
    }

    TerrainPatchSettings patchSettings{};
    bool regenerateTerrainRequested = true;
    bool havePatchCenter = false;
    double patchCenterLatDeg = 0.0;
    double patchCenterLonDeg = 0.0;

    bool renderOldEarthSphere = false;
    renderer.SetRenderOldEarthSphere(renderOldEarthSphere);

    double groundHeightRaw = 0.0;
    double groundHeightClamped = 0.0;
    bool tileLoadedAtPlane = false;
    TileKey currentTileKey = TerrainSystem::KeyForLatLon(sim.LatitudeDeg(), sim.LongitudeDeg());

    std::string terrainRuntimeError;

    bool previousBackspace = false;
    auto lastTime = std::chrono::high_resolution_clock::now();

    MSG msg{};
    bool running = true;

    while (running) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running = false;
                break;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        if (!running) {
            break;
        }

        const auto now = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        dt = std::clamp(dt, 0.0, 0.1);

        InputState input{};
        input.pitchUp = IsKeyDown('W');
        input.pitchDown = IsKeyDown('S');
        input.rollLeft = IsKeyDown('A');
        input.rollRight = IsKeyDown('D');
        input.yawLeft = IsKeyDown('Q');
        input.yawRight = IsKeyDown('E');
        input.throttleUp = IsKeyDown(VK_UP);
        input.throttleDown = IsKeyDown(VK_DOWN);
        input.altitudeUp = IsKeyDown('R');
        input.altitudeDown = IsKeyDown('F');

        const bool backspaceDown = IsKeyDown(VK_BACK);
        input.resetPressed = backspaceDown && !previousBackspace;
        previousBackspace = backspaceDown;

        sim.SetStart(editableStart);
        sim.Update(dt, input, static_cast<double>(timeScale));

        currentTileKey = TerrainSystem::KeyForLatLon(sim.LatitudeDeg(), sim.LongitudeDeg());
        if (terrainSystemReady) {
            groundHeightRaw = terrainSystem.SampleHeightMeters(sim.LatitudeDeg(), sim.LongitudeDeg(), &tileLoadedAtPlane);
        } else {
            groundHeightRaw = 0.0;
            tileLoadedAtPlane = false;
        }
        groundHeightClamped = std::max(groundHeightRaw, 0.0);

        if (terrainSystemReady) {
            bool shouldRegenerate = regenerateTerrainRequested || !havePatchCenter;
            if (!shouldRegenerate && havePatchCenter) {
                const double dLatRad = flight::DegToRad(sim.LatitudeDeg() - patchCenterLatDeg);
                const double dLonRad = flight::DegToRad(LonDeltaDeg(sim.LongitudeDeg(), patchCenterLonDeg));
                const double avgLatRad = flight::DegToRad(0.5 * (sim.LatitudeDeg() + patchCenterLatDeg));

                const double northMeters = dLatRad * flight::kEarthRadiusMeters;
                const double eastMeters = dLonRad * flight::kEarthRadiusMeters * std::max(0.01, std::cos(avgLatRad));
                const double distMeters = std::sqrt(northMeters * northMeters + eastMeters * eastMeters);
                const double threshold = std::clamp(patchSettings.patchSizeKm * 1000.0 * 0.20, 1000.0, 60000.0);
                shouldRegenerate = (distMeters >= threshold);
            }

            if (shouldRegenerate) {
                TerrainPatchBuildResult patch{};
                std::string buildError;
                if (flight::terrain::BuildTerrainPatch(
                        terrainSystem,
                        sim.LatitudeDeg(),
                        sim.LongitudeDeg(),
                        sim.PlaneEcef(),
                        patchSettings,
                        patch,
                        buildError)) {
                    std::string terrainUploadError;
                    if (renderer.SetTerrainMesh(patch.mesh, patch.anchorEcef, terrainUploadError)) {
                        patchCenterLatDeg = patch.centerLatDeg;
                        patchCenterLonDeg = patch.centerLonDeg;
                        havePatchCenter = true;
                        regenerateTerrainRequested = false;
                        terrainRuntimeError.clear();
                    } else {
                        terrainRuntimeError = "Terrain upload failed: " + terrainUploadError;
                    }
                } else {
                    terrainRuntimeError = "Terrain build failed: " + buildError;
                }
            }
        }

        renderer.BeginImGuiFrame();
        ImGui::NewFrame();

        ImGui::Begin("Flight HUD");
        ImGui::Text("Lat: %.6f deg", sim.LatitudeDeg());
        ImGui::Text("Lon: %.6f deg", sim.LongitudeDeg());
        ImGui::Text("Altitude: %.1f m", sim.AltitudeMeters());
        ImGui::Text("Speed: %.1f m/s", sim.SpeedMps());
        ImGui::Text("Heading: %.2f deg", sim.HeadingDeg());

        ImGui::Separator();
        ImGui::Text("Terrain Debug");
        ImGui::Text("Tile key: %s", currentTileKey.ToCompactString().c_str());
        ImGui::Text("Ground raw: %.2f m", groundHeightRaw);
        ImGui::Text("Ground clamped: %.2f m", groundHeightClamped);
        ImGui::Text("Tile loaded: %s", tileLoadedAtPlane ? "yes" : "no");
        ImGui::Text("Cache: %zu / %zu", terrainSystem.LoadedTileCount(), terrainSystem.CacheCapacity());

        if (ImGui::Checkbox("Render old Earth sphere", &renderOldEarthSphere)) {
            renderer.SetRenderOldEarthSphere(renderOldEarthSphere);
        }

        float patchSizeKm = static_cast<float>(patchSettings.patchSizeKm);
        if (ImGui::SliderFloat("Patch Size (km)", &patchSizeKm, 20.0f, 200.0f, "%.1f")) {
            patchSettings.patchSizeKm = static_cast<double>(patchSizeKm);
        }

        int gridRes = patchSettings.gridResolution;
        if (ImGui::SliderInt("Grid Resolution", &gridRes, 65, 513)) {
            if ((gridRes % 2) == 0) {
                gridRes += 1;
            }
            patchSettings.gridResolution = std::clamp(gridRes, 65, 513);
        }

        if (ImGui::Button("Regenerate Terrain")) {
            regenerateTerrainRequested = true;
        }

        if (!terrainInitError.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", terrainInitError.c_str());
        }
        if (!terrainRuntimeError.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s", terrainRuntimeError.c_str());
        }

        ImGui::Separator();
        ImGui::Text("Start/Respawn");
        ImGui::InputDouble("Start Latitude", &editableStart.latitudeDeg, 0.1, 1.0, "%.6f");
        ImGui::InputDouble("Start Longitude", &editableStart.longitudeDeg, 0.1, 1.0, "%.6f");
        ImGui::InputDouble("Start Altitude (m)", &editableStart.altitudeMeters, 10.0, 100.0, "%.1f");
        if (ImGui::Button("Respawn At Start")) {
            sim.SetStart(editableStart);
            sim.Respawn();
            regenerateTerrainRequested = true;
        }

        ImGui::SliderFloat("Time Scale", &timeScale, 1.0f, 20.0f, "%.1fx");

        ImGui::Separator();
        ImGui::Text("Controls");
        ImGui::Text("Pitch W/S, Roll A/D, Yaw Q/E");
        ImGui::Text("Throttle Up/Down arrows");
        ImGui::Text("Altitude R/F, Reset Backspace");
        ImGui::Text("Landmask: %s", renderer.HasLandmask() ? "loaded" : "fallback");
        ImGui::End();

        ImGui::Render();
        renderer.Render(sim, ImGui::GetDrawData());
    }

    renderer.Shutdown();
    g_renderer = nullptr;

    CoUninitialize();
    return 0;
}
