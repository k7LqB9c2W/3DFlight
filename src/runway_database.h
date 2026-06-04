#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

struct RunwayEndRecord {
    std::string airportIdent;
    std::string runwayIdent;
    std::string oppositeRunwayIdent;
    std::string surface;
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double elevationMeters = 0.0;
    double headingDeg = 0.0;
    double lengthMeters = 0.0;
    double widthMeters = 0.0;
    bool lowNumberedEnd = false;
};

class RunwayDatabase {
public:
    bool Load(const std::filesystem::path& path, std::string& error);

    [[nodiscard]] bool IsLoaded() const { return m_loaded; }
    [[nodiscard]] size_t AirportCount() const { return m_airportRunwayEnds.size(); }
    [[nodiscard]] size_t RunwayEndCount() const { return m_runwayEnds.size(); }

    [[nodiscard]] std::string ResolveAirportIdent(const std::string& ident) const;
    [[nodiscard]] const std::vector<int>& RunwayEndIndicesForAirport(const std::string& ident) const;
    [[nodiscard]] const RunwayEndRecord* GetRunwayEnd(int runwayEndIndex) const;
    [[nodiscard]] int DefaultRunwayEndIndexForAirport(const std::string& ident) const;

private:
    bool m_loaded = false;
    std::vector<RunwayEndRecord> m_runwayEnds;
    std::unordered_map<std::string, std::vector<int>> m_airportRunwayEnds;
};
