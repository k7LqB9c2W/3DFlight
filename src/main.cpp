#include <windows.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>

#include <imgui.h>
#include <imgui_impl_win32.h>

#include "d3d12_renderer.h"
#include "gltf_loader.h"
#include "mesh.h"
#include "sim.h"

using flight::D3D12Renderer;
using flight::FlightSim;
using flight::FlightStart;
using flight::InputState;
using flight::MeshData;

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

        renderer.BeginImGuiFrame();
        ImGui::NewFrame();

        ImGui::Begin("Flight HUD");
        ImGui::Text("Lat: %.6f deg", sim.LatitudeDeg());
        ImGui::Text("Lon: %.6f deg", sim.LongitudeDeg());
        ImGui::Text("Altitude: %.1f m", sim.AltitudeMeters());
        ImGui::Text("Speed: %.1f m/s", sim.SpeedMps());
        ImGui::Text("Heading: %.2f deg", sim.HeadingDeg());
        ImGui::Separator();
        ImGui::Text("Start/Respawn");
        ImGui::InputDouble("Start Latitude", &editableStart.latitudeDeg, 0.1, 1.0, "%.6f");
        ImGui::InputDouble("Start Longitude", &editableStart.longitudeDeg, 0.1, 1.0, "%.6f");
        ImGui::InputDouble("Start Altitude (m)", &editableStart.altitudeMeters, 10.0, 100.0, "%.1f");
        if (ImGui::Button("Respawn At Start")) {
            sim.SetStart(editableStart);
            sim.Respawn();
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
