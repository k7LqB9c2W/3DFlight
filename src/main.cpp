#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include <imgui.h>
#include <imgui_impl_win32.h>
#include <nlohmann/json.hpp>

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
constexpr double kPi = 3.14159265358979323846;

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

double WrapHours24(double hours) {
    while (hours >= 24.0) {
        hours -= 24.0;
    }
    while (hours < 0.0) {
        hours += 24.0;
    }
    return hours;
}

void AdvanceLocalSolarClock(float& localSolarTimeHours, int& dayOfYear, double deltaHours) {
    double hours = static_cast<double>(localSolarTimeHours) + deltaHours;
    int day = std::clamp(dayOfYear, 1, 365);
    while (hours >= 24.0) {
        hours -= 24.0;
        day += 1;
        if (day > 365) {
            day = 1;
        }
    }
    while (hours < 0.0) {
        hours += 24.0;
        day -= 1;
        if (day < 1) {
            day = 365;
        }
    }
    localSolarTimeHours = static_cast<float>(hours);
    dayOfYear = day;
}

flight::Double3 ComputeSunDirectionFromLocalSolarTime(
    double latitudeDeg,
    double longitudeDeg,
    int dayOfYear,
    double localSolarTimeHours) {
    const int clampedDay = std::clamp(dayOfYear, 1, 365);
    const double wrappedHours = WrapHours24(localSolarTimeHours);

    const double latRad = flight::DegToRad(latitudeDeg);
    const double lonRad = flight::DegToRad(longitudeDeg);
    const double declinationRad =
        flight::DegToRad(23.44) * std::sin((2.0 * kPi / 365.0) * (static_cast<double>(clampedDay) - 81.0));
    const double hourAngleRad = flight::DegToRad((wrappedHours - 12.0) * 15.0);

    const double cosDecl = std::cos(declinationRad);
    const double sinDecl = std::sin(declinationRad);
    const double cosLat = std::cos(latRad);
    const double sinLat = std::sin(latRad);
    const double cosHour = std::cos(hourAngleRad);
    const double sinHour = std::sin(hourAngleRad);

    // Local ENU sun direction; east sign keeps morning sun in the east and afternoon sun in the west.
    const double east = -cosDecl * sinHour;
    const double north = cosLat * sinDecl - sinLat * cosDecl * cosHour;
    const double up = sinLat * sinDecl + cosLat * cosDecl * cosHour;

    flight::Double3 eastAxis, northAxis, upAxis;
    flight::BuildEnuBasis(latRad, lonRad, eastAxis, northAxis, upAxis);
    return flight::Normalize(eastAxis * east + northAxis * north + upAxis * up);
}

struct GraphicsTuningConfig {
    float patchSizeKm = 80.0f;
    int gridResolution = 257;
    float lodDistanceScale = 1.0f;
    float lodResolutionScale = 1.0f;
    float nearToMidMultiplier = 2.6f;
    float midToFarMultiplier = 2.6f;
    float farFieldRadiusKm = 1200.0f;
    float verticalExaggeration = 1.0f;

    float colorHeightMaxMeters = 6000.0f;
    float lodTransitionWidthMeters = 5000.0f;
    float hazeStrength = 0.40f;
    float hazeAltitudeRangeMeters = 16000.0f;
    float colorContrast = 1.0f;
    float slopeShadingStrength = 0.35f;
    float specularStrength = 0.14f;
    float lodSeamBlendStrength = 0.45f;
    float aerialPerspectiveDepthMeters = 32000.0f;
};

void SanitizeGraphicsTuning(GraphicsTuningConfig& cfg) {
    cfg.patchSizeKm = std::clamp(cfg.patchSizeKm, 20.0f, 250.0f);
    cfg.gridResolution = std::clamp(cfg.gridResolution, 65, 513);
    if ((cfg.gridResolution % 2) == 0) {
        cfg.gridResolution += 1;
    }
    cfg.lodDistanceScale = std::clamp(cfg.lodDistanceScale, 0.35f, 4.0f);
    cfg.lodResolutionScale = std::clamp(cfg.lodResolutionScale, 0.35f, 2.5f);
    cfg.nearToMidMultiplier = std::clamp(cfg.nearToMidMultiplier, 1.4f, 4.5f);
    cfg.midToFarMultiplier = std::clamp(cfg.midToFarMultiplier, 1.4f, 4.5f);
    cfg.farFieldRadiusKm = std::clamp(cfg.farFieldRadiusKm, 120.0f, 4000.0f);
    cfg.verticalExaggeration = std::clamp(cfg.verticalExaggeration, 0.5f, 3.0f);

    cfg.colorHeightMaxMeters = std::clamp(cfg.colorHeightMaxMeters, 1000.0f, 20000.0f);
    cfg.lodTransitionWidthMeters = std::clamp(cfg.lodTransitionWidthMeters, 250.0f, 50000.0f);
    cfg.hazeStrength = std::clamp(cfg.hazeStrength, 0.0f, 2.0f);
    cfg.hazeAltitudeRangeMeters = std::clamp(cfg.hazeAltitudeRangeMeters, 1000.0f, 80000.0f);
    cfg.colorContrast = std::clamp(cfg.colorContrast, 0.2f, 3.0f);
    cfg.slopeShadingStrength = std::clamp(cfg.slopeShadingStrength, 0.0f, 2.0f);
    cfg.specularStrength = std::clamp(cfg.specularStrength, 0.0f, 1.0f);
    cfg.lodSeamBlendStrength = std::clamp(cfg.lodSeamBlendStrength, 0.0f, 2.0f);
    cfg.aerialPerspectiveDepthMeters = std::clamp(cfg.aerialPerspectiveDepthMeters, 5000.0f, 1000000.0f);
}

GraphicsTuningConfig MakeRealisticTuningPreset() {
    GraphicsTuningConfig cfg;
    cfg.patchSizeKm = 100.0f;
    cfg.gridResolution = 289;
    cfg.lodDistanceScale = 1.2f;
    cfg.lodResolutionScale = 1.2f;
    cfg.nearToMidMultiplier = 2.25f;
    cfg.midToFarMultiplier = 2.15f;
    cfg.farFieldRadiusKm = 1800.0f;
    cfg.verticalExaggeration = 1.0f;
    cfg.colorHeightMaxMeters = 7200.0f;
    cfg.lodTransitionWidthMeters = 7000.0f;
    cfg.hazeStrength = 0.30f;
    cfg.hazeAltitudeRangeMeters = 18000.0f;
    cfg.colorContrast = 1.10f;
    cfg.slopeShadingStrength = 0.42f;
    cfg.specularStrength = 0.10f;
    cfg.lodSeamBlendStrength = 0.42f;
    cfg.aerialPerspectiveDepthMeters = 65000.0f;
    SanitizeGraphicsTuning(cfg);
    return cfg;
}

GraphicsTuningConfig MakeCinematicTuningPreset() {
    GraphicsTuningConfig cfg = MakeRealisticTuningPreset();
    cfg.verticalExaggeration = 1.15f;
    cfg.colorContrast = 1.25f;
    cfg.hazeStrength = 0.42f;
    cfg.hazeAltitudeRangeMeters = 14000.0f;
    cfg.specularStrength = 0.18f;
    cfg.aerialPerspectiveDepthMeters = 55000.0f;
    SanitizeGraphicsTuning(cfg);
    return cfg;
}

GraphicsTuningConfig MakeCrispTerrainTuningPreset() {
    GraphicsTuningConfig cfg = MakeRealisticTuningPreset();
    cfg.lodDistanceScale = 1.35f;
    cfg.lodResolutionScale = 1.45f;
    cfg.verticalExaggeration = 1.18f;
    cfg.nearToMidMultiplier = 2.05f;
    cfg.midToFarMultiplier = 1.95f;
    cfg.hazeStrength = 0.16f;
    cfg.colorContrast = 1.35f;
    cfg.slopeShadingStrength = 0.58f;
    cfg.aerialPerspectiveDepthMeters = 90000.0f;
    SanitizeGraphicsTuning(cfg);
    return cfg;
}

void ApplyGraphicsTuning(
    const GraphicsTuningConfig& cfg,
    TerrainPatchSettings& patchSettings,
    D3D12Renderer& renderer) {
    patchSettings.patchSizeKm = static_cast<double>(cfg.patchSizeKm);
    patchSettings.gridResolution = cfg.gridResolution;
    patchSettings.lodDistanceScale = static_cast<double>(cfg.lodDistanceScale);
    patchSettings.lodResolutionScale = static_cast<double>(cfg.lodResolutionScale);
    patchSettings.nearToMidMultiplier = static_cast<double>(cfg.nearToMidMultiplier);
    patchSettings.midToFarMultiplier = static_cast<double>(cfg.midToFarMultiplier);
    patchSettings.farFieldRadiusKm = static_cast<double>(cfg.farFieldRadiusKm);
    patchSettings.verticalExaggeration = static_cast<double>(cfg.verticalExaggeration);

    D3D12Renderer::TerrainVisualSettings visual{};
    visual.colorHeightMaxMeters = cfg.colorHeightMaxMeters;
    visual.lodTransitionWidthMeters = cfg.lodTransitionWidthMeters;
    visual.midRingMultiplier = cfg.nearToMidMultiplier;
    visual.farRingMultiplier = cfg.midToFarMultiplier;
    visual.hazeStrength = cfg.hazeStrength;
    visual.hazeAltitudeRangeMeters = cfg.hazeAltitudeRangeMeters;
    visual.colorContrast = cfg.colorContrast;
    visual.slopeShadingStrength = cfg.slopeShadingStrength;
    visual.specularStrength = cfg.specularStrength;
    visual.lodSeamBlendStrength = cfg.lodSeamBlendStrength;
    renderer.SetTerrainVisualSettings(visual);
    renderer.SetAerialPerspectiveDepthMeters(cfg.aerialPerspectiveDepthMeters);
}

bool LoadGraphicsTuningConfig(const std::filesystem::path& path, GraphicsTuningConfig& outCfg, std::string& error) {
    error.clear();
    std::ifstream in(path);
    if (!in) {
        error = "Failed to open tuning file: " + path.string();
        return false;
    }

    nlohmann::json j;
    try {
        in >> j;
    } catch (const std::exception& ex) {
        error = "Failed to parse tuning JSON: " + std::string(ex.what());
        return false;
    }

    auto readFloat = [&j](const char* key, float& value) {
        if (j.contains(key) && j[key].is_number()) {
            value = j[key].get<float>();
        }
    };
    auto readInt = [&j](const char* key, int& value) {
        if (j.contains(key) && j[key].is_number_integer()) {
            value = j[key].get<int>();
        }
    };

    readFloat("patch_size_km", outCfg.patchSizeKm);
    readInt("grid_resolution", outCfg.gridResolution);
    readFloat("lod_distance_scale", outCfg.lodDistanceScale);
    readFloat("lod_resolution_scale", outCfg.lodResolutionScale);
    readFloat("near_to_mid_multiplier", outCfg.nearToMidMultiplier);
    readFloat("mid_to_far_multiplier", outCfg.midToFarMultiplier);
    readFloat("far_field_radius_km", outCfg.farFieldRadiusKm);
    readFloat("vertical_exaggeration", outCfg.verticalExaggeration);

    readFloat("color_height_max_m", outCfg.colorHeightMaxMeters);
    readFloat("lod_transition_width_m", outCfg.lodTransitionWidthMeters);
    readFloat("haze_strength", outCfg.hazeStrength);
    readFloat("haze_altitude_range_m", outCfg.hazeAltitudeRangeMeters);
    readFloat("color_contrast", outCfg.colorContrast);
    readFloat("slope_shading_strength", outCfg.slopeShadingStrength);
    readFloat("specular_strength", outCfg.specularStrength);
    readFloat("lod_seam_blend_strength", outCfg.lodSeamBlendStrength);
    readFloat("aerial_perspective_depth_m", outCfg.aerialPerspectiveDepthMeters);

    SanitizeGraphicsTuning(outCfg);
    return true;
}

bool SaveGraphicsTuningConfig(const std::filesystem::path& path, const GraphicsTuningConfig& cfg, std::string& error) {
    error.clear();
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
        error = "Failed to create config directory: " + ec.message();
        return false;
    }

    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        error = "Failed to write tuning file: " + path.string();
        return false;
    }

    nlohmann::json j;
    j["patch_size_km"] = cfg.patchSizeKm;
    j["grid_resolution"] = cfg.gridResolution;
    j["lod_distance_scale"] = cfg.lodDistanceScale;
    j["lod_resolution_scale"] = cfg.lodResolutionScale;
    j["near_to_mid_multiplier"] = cfg.nearToMidMultiplier;
    j["mid_to_far_multiplier"] = cfg.midToFarMultiplier;
    j["far_field_radius_km"] = cfg.farFieldRadiusKm;
    j["vertical_exaggeration"] = cfg.verticalExaggeration;
    j["color_height_max_m"] = cfg.colorHeightMaxMeters;
    j["lod_transition_width_m"] = cfg.lodTransitionWidthMeters;
    j["haze_strength"] = cfg.hazeStrength;
    j["haze_altitude_range_m"] = cfg.hazeAltitudeRangeMeters;
    j["color_contrast"] = cfg.colorContrast;
    j["slope_shading_strength"] = cfg.slopeShadingStrength;
    j["specular_strength"] = cfg.specularStrength;
    j["lod_seam_blend_strength"] = cfg.lodSeamBlendStrength;
    j["aerial_perspective_depth_m"] = cfg.aerialPerspectiveDepthMeters;

    out << j.dump(2) << "\n";
    return true;
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
    float simTimeScale = 1.0f;
    float localSolarTimeHours = 14.0f;
    int dayOfYear = 172;
    bool autoTimeOfDay = false;
    float timeOfDayRateHoursPerSecond = 0.5f;

    TerrainSystem terrainSystem;
    bool terrainSystemReady = false;
    std::string terrainInitError;
    if (!terrainSystem.Initialize(std::filesystem::path("assets") / "etopo" / "tiles", 49, terrainInitError)) {
        terrainInitError = "Terrain init failed: " + terrainInitError;
    } else {
        terrainSystemReady = true;
    }

    TerrainPatchSettings patchSettings{};
    GraphicsTuningConfig graphicsTuning = MakeRealisticTuningPreset();
    const std::filesystem::path graphicsConfigPath = std::filesystem::path("config") / "graphics_tuning.json";
    std::string graphicsConfigStatus;
    {
        std::error_code existsEc;
        const bool configExists = std::filesystem::exists(graphicsConfigPath, existsEc) && !existsEc;
        if (configExists) {
            std::string loadError;
            if (LoadGraphicsTuningConfig(graphicsConfigPath, graphicsTuning, loadError)) {
                graphicsConfigStatus = "Loaded " + graphicsConfigPath.string();
            } else {
                graphicsConfigStatus = "Config load failed: " + loadError;
            }
        } else {
            std::string saveError;
            SanitizeGraphicsTuning(graphicsTuning);
            if (SaveGraphicsTuningConfig(graphicsConfigPath, graphicsTuning, saveError)) {
                graphicsConfigStatus = "Created default " + graphicsConfigPath.string();
            } else {
                graphicsConfigStatus = "Config create failed: " + saveError;
            }
        }
    }
    ApplyGraphicsTuning(graphicsTuning, patchSettings, renderer);
    std::filesystem::file_time_type graphicsConfigWriteTime{};
    bool hasGraphicsConfigWriteTime = false;
    {
        std::error_code timeEc;
        const auto t = std::filesystem::last_write_time(graphicsConfigPath, timeEc);
        if (!timeEc) {
            graphicsConfigWriteTime = t;
            hasGraphicsConfigWriteTime = true;
        }
    }
    double graphicsHotReloadAccumulator = 0.0;

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

        graphicsHotReloadAccumulator += dt;
        if (graphicsHotReloadAccumulator >= 1.0) {
            graphicsHotReloadAccumulator = 0.0;
            std::error_code timeEc;
            const auto t = std::filesystem::last_write_time(graphicsConfigPath, timeEc);
            if (!timeEc) {
                const bool changed = !hasGraphicsConfigWriteTime || (t != graphicsConfigWriteTime);
                if (changed) {
                    GraphicsTuningConfig reloaded = graphicsTuning;
                    std::string loadError;
                    if (LoadGraphicsTuningConfig(graphicsConfigPath, reloaded, loadError)) {
                        graphicsTuning = reloaded;
                        ApplyGraphicsTuning(graphicsTuning, patchSettings, renderer);
                        regenerateTerrainRequested = true;
                        graphicsConfigStatus = "Hot reloaded " + graphicsConfigPath.string();
                    } else {
                        graphicsConfigStatus = "Hot reload failed: " + loadError;
                    }
                    graphicsConfigWriteTime = t;
                    hasGraphicsConfigWriteTime = true;
                }
            }
        }

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
        sim.Update(dt, input, static_cast<double>(simTimeScale));

        if (autoTimeOfDay) {
            AdvanceLocalSolarClock(localSolarTimeHours, dayOfYear, static_cast<double>(timeOfDayRateHoursPerSecond) * dt);
        }
        localSolarTimeHours = static_cast<float>(WrapHours24(localSolarTimeHours));
        dayOfYear = std::clamp(dayOfYear, 1, 365);

        const flight::Double3 sunDir =
            ComputeSunDirectionFromLocalSolarTime(sim.LatitudeDeg(), sim.LongitudeDeg(), dayOfYear, localSolarTimeHours);
        renderer.SetSunDirection(sunDir);
        const double sunElevationDeg = flight::RadToDeg(std::asin(std::clamp(flight::Dot(sunDir, sim.UpEcef()), -1.0, 1.0)));

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

        ImGui::Separator();
        ImGui::Text("Time Of Day");
        ImGui::Checkbox("Auto Time Advance", &autoTimeOfDay);
        if (ImGui::SliderFloat("Local Solar Time (h)", &localSolarTimeHours, 0.0f, 24.0f, "%.2f")) {
            localSolarTimeHours = static_cast<float>(WrapHours24(localSolarTimeHours));
        }
        ImGui::SliderInt("Day Of Year", &dayOfYear, 1, 365);
        ImGui::SliderFloat("Time Rate (h/s)", &timeOfDayRateHoursPerSecond, -4.0f, 20.0f, "%.2f");
        if (ImGui::Button("Sunrise")) {
            localSolarTimeHours = 6.0f;
        }
        ImGui::SameLine();
        if (ImGui::Button("Noon")) {
            localSolarTimeHours = 12.0f;
        }
        ImGui::SameLine();
        if (ImGui::Button("Sunset")) {
            localSolarTimeHours = 18.0f;
        }
        ImGui::Text("Sun Elevation: %.2f deg", sunElevationDeg);

        ImGui::Separator();
        if (ImGui::Checkbox("Render old Earth sphere", &renderOldEarthSphere)) {
            renderer.SetRenderOldEarthSphere(renderOldEarthSphere);
        }
        ImGui::TextUnformatted("Use Graphics Tuning panel for LOD/fog/detail tuning.");

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

        ImGui::SliderFloat("Sim Time Scale", &simTimeScale, 1.0f, 20.0f, "%.1fx");

        ImGui::Separator();
        ImGui::Text("Controls");
        ImGui::Text("Pitch W/S, Roll A/D, Yaw Q/E");
        ImGui::Text("Throttle Up/Down arrows");
        ImGui::Text("Altitude R/F, Reset Backspace");
        ImGui::Text("Landmask: %s", renderer.HasLandmask() ? "loaded" : "fallback");
        ImGui::End();

        ImGui::Begin("Graphics Tuning");
        bool meshTuningChanged = false;
        bool visualTuningChanged = false;

        if (ImGui::SliderFloat("Patch Size (km)", &graphicsTuning.patchSizeKm, 20.0f, 250.0f, "%.1f")) {
            meshTuningChanged = true;
        }
        if (ImGui::SliderInt("Grid Resolution", &graphicsTuning.gridResolution, 65, 513)) {
            if ((graphicsTuning.gridResolution % 2) == 0) {
                graphicsTuning.gridResolution += 1;
            }
            meshTuningChanged = true;
        }
        if (ImGui::SliderFloat("LOD Distance Scale", &graphicsTuning.lodDistanceScale, 0.35f, 4.0f, "%.2f")) {
            meshTuningChanged = true;
        }
        if (ImGui::SliderFloat("LOD Resolution Scale", &graphicsTuning.lodResolutionScale, 0.35f, 2.5f, "%.2f")) {
            meshTuningChanged = true;
        }
        if (ImGui::SliderFloat("Near->Mid Mult", &graphicsTuning.nearToMidMultiplier, 1.4f, 4.5f, "%.2f")) {
            meshTuningChanged = true;
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Mid->Far Mult", &graphicsTuning.midToFarMultiplier, 1.4f, 4.5f, "%.2f")) {
            meshTuningChanged = true;
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Far Field Radius (km)", &graphicsTuning.farFieldRadiusKm, 120.0f, 4000.0f, "%.0f")) {
            meshTuningChanged = true;
        }
        if (ImGui::SliderFloat("Vertical Exaggeration", &graphicsTuning.verticalExaggeration, 0.5f, 3.0f, "%.2f")) {
            meshTuningChanged = true;
        }

        ImGui::Separator();
        if (ImGui::SliderFloat("Color Height Max (m)", &graphicsTuning.colorHeightMaxMeters, 1000.0f, 20000.0f, "%.0f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("LOD Transition Width (m)", &graphicsTuning.lodTransitionWidthMeters, 250.0f, 50000.0f, "%.0f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Haze Strength", &graphicsTuning.hazeStrength, 0.0f, 2.0f, "%.2f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Haze Altitude Range (m)", &graphicsTuning.hazeAltitudeRangeMeters, 1000.0f, 80000.0f, "%.0f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Color Contrast", &graphicsTuning.colorContrast, 0.2f, 3.0f, "%.2f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Slope Shading", &graphicsTuning.slopeShadingStrength, 0.0f, 2.0f, "%.2f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Specular Strength", &graphicsTuning.specularStrength, 0.0f, 1.0f, "%.2f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("LOD Seam Blend", &graphicsTuning.lodSeamBlendStrength, 0.0f, 2.0f, "%.2f")) {
            visualTuningChanged = true;
        }
        if (ImGui::SliderFloat("Aerial Depth (m)", &graphicsTuning.aerialPerspectiveDepthMeters, 5000.0f, 1000000.0f, "%.0f")) {
            visualTuningChanged = true;
        }

        ImGui::Separator();
        if (ImGui::Button("Preset: Realistic")) {
            graphicsTuning = MakeRealisticTuningPreset();
            meshTuningChanged = true;
            visualTuningChanged = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Preset: Cinematic")) {
            graphicsTuning = MakeCinematicTuningPreset();
            meshTuningChanged = true;
            visualTuningChanged = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Preset: Crisp")) {
            graphicsTuning = MakeCrispTerrainTuningPreset();
            meshTuningChanged = true;
            visualTuningChanged = true;
        }

        if (ImGui::Button("Regenerate Terrain")) {
            regenerateTerrainRequested = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Save Tuning")) {
            SanitizeGraphicsTuning(graphicsTuning);
            std::string saveError;
            if (SaveGraphicsTuningConfig(graphicsConfigPath, graphicsTuning, saveError)) {
                graphicsConfigStatus = "Saved " + graphicsConfigPath.string();
                std::error_code timeEc;
                const auto t = std::filesystem::last_write_time(graphicsConfigPath, timeEc);
                if (!timeEc) {
                    graphicsConfigWriteTime = t;
                    hasGraphicsConfigWriteTime = true;
                }
            } else {
                graphicsConfigStatus = "Save failed: " + saveError;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Reload Tuning")) {
            GraphicsTuningConfig loaded = graphicsTuning;
            std::string loadError;
            if (LoadGraphicsTuningConfig(graphicsConfigPath, loaded, loadError)) {
                graphicsTuning = loaded;
                meshTuningChanged = true;
                visualTuningChanged = true;
                graphicsConfigStatus = "Reloaded " + graphicsConfigPath.string();
                std::error_code timeEc;
                const auto t = std::filesystem::last_write_time(graphicsConfigPath, timeEc);
                if (!timeEc) {
                    graphicsConfigWriteTime = t;
                    hasGraphicsConfigWriteTime = true;
                }
            } else {
                graphicsConfigStatus = "Reload failed: " + loadError;
            }
        }

        if (meshTuningChanged || visualTuningChanged) {
            SanitizeGraphicsTuning(graphicsTuning);
            ApplyGraphicsTuning(graphicsTuning, patchSettings, renderer);
            if (meshTuningChanged) {
                regenerateTerrainRequested = true;
            }
        }

        ImGui::Separator();
        ImGui::Text("Config: %s", graphicsConfigPath.string().c_str());
        if (!graphicsConfigStatus.empty()) {
            ImGui::TextWrapped("%s", graphicsConfigStatus.c_str());
        }
        ImGui::TextUnformatted("Mesh sliders trigger terrain regeneration. Visual sliders apply live.");
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
