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
using flight::terrain::TerrainSampleDebug;
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

double WrapLonDeg(double lonDeg) {
    while (lonDeg >= 180.0) {
        lonDeg -= 360.0;
    }
    while (lonDeg < -180.0) {
        lonDeg += 360.0;
    }
    return lonDeg;
}

void OffsetLatLonMeters(double latDeg, double lonDeg, double northMeters, double eastMeters, double& outLatDeg, double& outLonDeg) {
    const double latRad = flight::DegToRad(latDeg);
    const double dLatDeg = flight::RadToDeg(northMeters / flight::kEarthRadiusMeters);
    const double cosLat = std::max(0.01, std::cos(latRad));
    const double dLonDeg = flight::RadToDeg(eastMeters / (flight::kEarthRadiusMeters * cosLat));
    outLatDeg = std::clamp(latDeg + dLatDeg, -89.999, 89.999);
    outLonDeg = WrapLonDeg(lonDeg + dLonDeg);
}

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int showCmd) {
    CoInitializeEx(nullptr, COINITBASE_MULTITHREADED);

    const wchar_t* kClassName = L"3DFlightClass";

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
        L"3DFlight",
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
    if (!terrainSystem.Initialize(std::filesystem::path("assets") / "etopo" / "tiles", 49, terrainInitError)) {
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
    TerrainSampleDebug terrainSampleDebug{};
    TileKey currentTileKey = TerrainSystem::KeyForLatLon(sim.LatitudeDeg(), sim.LongitudeDeg());
    double terrainPatchMinRaw = 0.0;
    double terrainPatchMaxRaw = 0.0;
    float debugProbeStepKm = 10.0f;

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

        if (terrainSystemReady) {
            terrainSystem.PrefetchAround(sim.LatitudeDeg(), sim.LongitudeDeg(), sim.HeadingDeg(), sim.SpeedMps(), 2);
        }

        if (terrainSystemReady) {
            groundHeightRaw = terrainSystem.SampleHeightMetersDebug(sim.LatitudeDeg(), sim.LongitudeDeg(), terrainSampleDebug);
            currentTileKey = terrainSampleDebug.key;
            tileLoadedAtPlane = terrainSampleDebug.tileLoaded;
        } else {
            groundHeightRaw = 0.0;
            tileLoadedAtPlane = false;
            terrainSampleDebug = {};
            currentTileKey = TerrainSystem::KeyForLatLon(sim.LatitudeDeg(), sim.LongitudeDeg());
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
                    if (renderer.SetTerrainMesh(patch.mesh, patch.anchorEcef, patch.renderParams, terrainUploadError)) {
                        patchCenterLatDeg = patch.centerLatDeg;
                        patchCenterLonDeg = patch.centerLonDeg;
                        terrainPatchMinRaw = patch.minHeightRaw;
                        terrainPatchMaxRaw = patch.maxHeightRaw;
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

        bool atmosphereEnabled = renderer.IsAtmosphereEnabled();
        if (ImGui::Checkbox("Atmosphere Enabled", &atmosphereEnabled)) {
            renderer.SetAtmosphereEnabled(atmosphereEnabled);
        }

        bool multipleScatteringEnabled = renderer.IsMultipleScatteringEnabled();
        if (ImGui::Checkbox("Multiple Scattering", &multipleScatteringEnabled)) {
            renderer.SetMultipleScatteringEnabled(multipleScatteringEnabled);
        }

        float atmosphereExposure = renderer.AtmosphereExposure();
        if (ImGui::SliderFloat("Atmosphere Exposure", &atmosphereExposure, 0.25f, 3.0f, "%.2f")) {
            renderer.SetAtmosphereExposure(atmosphereExposure);
        }

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

        ImGui::Begin("Terrain Probe");
        if (!terrainSystemReady) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Terrain system is not ready.");
        } else {
            const std::string expectedPath = terrainSampleDebug.expectedTilePath.string();
            const std::string resolvedPath = terrainSampleDebug.tilePathFound ? terrainSampleDebug.resolvedTilePath.string() : "(missing)";

            ImGui::Text("Sample Lat/Lon: %.6f, %.6f", sim.LatitudeDeg(), sim.LongitudeDeg());
            ImGui::Text("Tile key: %s", terrainSampleDebug.key.ToCompactString().c_str());
            ImGui::Text("Cache hit: %s", terrainSampleDebug.cacheHit ? "yes" : "no");
            ImGui::Text("Tile available: %s", terrainSampleDebug.tileLoaded ? "yes" : "no");
            ImGui::Text("Loaded this frame: %s", terrainSampleDebug.tileLoadedThisCall ? "yes" : "no");
            ImGui::Text("Expected file: %s", expectedPath.c_str());
            ImGui::Text("Resolved file: %s", resolvedPath.c_str());

            const auto& probe = terrainSampleDebug.tileSample;
            ImGui::Separator();
            ImGui::Text("Map success: %s", probe.mapSucceeded ? "yes" : "no");
            ImGui::Text("Map mode: %s", flight::terrain::PixelMappingModeToString(probe.mappingMode));
            ImGui::Text("Mapped lon: %.6f (wrap %+g)", probe.mappedLonDeg, probe.lonWrapAppliedDeg);
            ImGui::Text("Raw pixel: (%.3f, %.3f)", probe.rawPixelX, probe.rawPixelY);
            ImGui::Text("Clamped pixel: (%.3f, %.3f)", probe.pixelX, probe.pixelY);
            ImGui::Text("Texels: (%d,%d)-(%d,%d)", probe.x0, probe.y0, probe.x1, probe.y1);
            ImGui::Text("Bilinear frac: tx=%.3f ty=%.3f", probe.tx, probe.ty);
            ImGui::Text("Samples: h00=%.1f h10=%.1f h01=%.1f h11=%.1f", probe.h00, probe.h10, probe.h01, probe.h11);
            ImGui::Text("Tile bounds lat[%.6f, %.6f] lon[%.6f, %.6f]", probe.latSouth, probe.latNorth, probe.lonWest, probe.lonEast);
            ImGui::Text("Tile raster: %d x %d", probe.width, probe.height);
            ImGui::Text("Patch raw min/max: %.1f / %.1f m", terrainPatchMinRaw, terrainPatchMaxRaw);

            ImGui::Separator();
            ImGui::SliderFloat("Probe Step (km)", &debugProbeStepKm, 1.0f, 100.0f, "%.1f");
            ImGui::Text("3x3 raw heights around plane (north rows, west->east cols)");
            for (int row = 1; row >= -1; --row) {
                double rowSamples[3]{};
                for (int col = -1; col <= 1; ++col) {
                    double sampleLatDeg = 0.0;
                    double sampleLonDeg = 0.0;
                    OffsetLatLonMeters(
                        sim.LatitudeDeg(),
                        sim.LongitudeDeg(),
                        static_cast<double>(row) * static_cast<double>(debugProbeStepKm) * 1000.0,
                        static_cast<double>(col) * static_cast<double>(debugProbeStepKm) * 1000.0,
                        sampleLatDeg,
                        sampleLonDeg);
                    rowSamples[col + 1] = terrainSystem.SampleHeightMeters(sampleLatDeg, sampleLonDeg);
                }
                ImGui::Text(
                    "%+5.1f km N: %8.1f  %8.1f  %8.1f",
                    static_cast<double>(row) * static_cast<double>(debugProbeStepKm),
                    rowSamples[0],
                    rowSamples[1],
                    rowSamples[2]);
            }
            ImGui::Text("Columns are west / center / east at %+0.1f km.", debugProbeStepKm);
        }
        ImGui::End();

        ImGui::Render();
        renderer.Render(sim, ImGui::GetDrawData());
    }

    renderer.Shutdown();
    g_renderer = nullptr;

    CoUninitialize();
    return 0;
}
