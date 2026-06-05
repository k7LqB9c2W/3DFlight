#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "sim.h"

namespace flight {

struct FgSoundDebugInfo {
    std::string name;
    std::string path;
    bool playing = false;
    bool condition = false;
    float volume = 0.0f;
    float pitch = 1.0f;
};

struct FgPropertyDebugInfo {
    std::string path;
    std::string value;
};

class FgRuntime {
public:
    FgRuntime();
    ~FgRuntime();

    FgRuntime(const FgRuntime&) = delete;
    FgRuntime& operator=(const FgRuntime&) = delete;

    bool Initialize(const std::filesystem::path& aircraftRoot, std::string& error);
    void Shutdown();
    void Reset();

    void SetEnabled(bool enabled);
    [[nodiscard]] bool IsEnabled() const { return m_enabled; }
    [[nodiscard]] bool IsReady() const;
    [[nodiscard]] const std::string& Status() const { return m_status; }

    void UpdateFromSim(const FlightSim& sim, bool aircraftActive, bool cockpitView, double dtSeconds);

    [[nodiscard]] std::vector<FgSoundDebugInfo> SoundDebugSnapshot() const;
    [[nodiscard]] std::vector<FgPropertyDebugInfo> FindProperties(const std::string& query, size_t limit) const;
    bool SetPropertyFromUi(const std::string& path, const std::string& valueText);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    bool m_enabled = true;
    std::string m_status = "FlightGear audio: not initialized";
};

} // namespace flight
