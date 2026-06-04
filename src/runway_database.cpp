#include "runway_database.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFeetToMeters = 0.3048;

const std::vector<int> kEmptyIndices;

std::string Trim(std::string value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::string ToUpperAscii(std::string value) {
    for (char& ch : value) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::vector<std::string> ParseCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool inQuotes = false;

    for (size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            if (inQuotes && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"');
                ++i;
            } else {
                inQuotes = !inQuotes;
            }
        } else if (ch == ',' && !inQuotes) {
            fields.push_back(field);
            field.clear();
        } else {
            field.push_back(ch);
        }
    }

    fields.push_back(field);
    return fields;
}

double ParseDoubleOr(const std::vector<std::string>& fields, const std::unordered_map<std::string, size_t>& columns, const char* key, double fallback) {
    const auto it = columns.find(key);
    if (it == columns.end() || it->second >= fields.size()) {
        return fallback;
    }

    const std::string value = Trim(fields[it->second]);
    if (value.empty()) {
        return fallback;
    }

    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str()) {
        return fallback;
    }
    return parsed;
}

std::string ParseStringOr(const std::vector<std::string>& fields, const std::unordered_map<std::string, size_t>& columns, const char* key) {
    const auto it = columns.find(key);
    if (it == columns.end() || it->second >= fields.size()) {
        return {};
    }
    return Trim(fields[it->second]);
}

bool HasDouble(const std::vector<std::string>& fields, const std::unordered_map<std::string, size_t>& columns, const char* key) {
    const auto it = columns.find(key);
    return it != columns.end() && it->second < fields.size() && !Trim(fields[it->second]).empty();
}

double WrapDeg360(double degrees) {
    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped;
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

double BearingDegBetween(double latADeg, double lonADeg, double latBDeg, double lonBDeg) {
    const double latA = latADeg * (kPi / 180.0);
    const double latB = latBDeg * (kPi / 180.0);
    const double dLon = LonDeltaDeg(lonBDeg, lonADeg) * (kPi / 180.0);
    const double y = std::sin(dLon) * std::cos(latB);
    const double x = std::cos(latA) * std::sin(latB) - std::sin(latA) * std::cos(latB) * std::cos(dLon);
    return WrapDeg360(std::atan2(y, x) * (180.0 / kPi));
}

bool IsValidLatLon(double latDeg, double lonDeg) {
    return std::isfinite(latDeg) && std::isfinite(lonDeg) && latDeg >= -90.0 && latDeg <= 90.0 && lonDeg >= -180.0 && lonDeg <= 180.0;
}

RunwayEndRecord MakeRunwayEnd(
    const std::string& airportIdent,
    const std::string& runwayIdent,
    const std::string& oppositeRunwayIdent,
    const std::string& surface,
    double latDeg,
    double lonDeg,
    double elevationFeet,
    double headingDeg,
    double lengthFeet,
    double widthFeet,
    bool lowNumberedEnd) {
    RunwayEndRecord end;
    end.airportIdent = airportIdent;
    end.runwayIdent = runwayIdent;
    end.oppositeRunwayIdent = oppositeRunwayIdent;
    end.surface = surface;
    end.latitudeDeg = latDeg;
    end.longitudeDeg = lonDeg;
    end.elevationMeters = elevationFeet * kFeetToMeters;
    end.headingDeg = headingDeg;
    end.lengthMeters = lengthFeet * kFeetToMeters;
    end.widthMeters = widthFeet * kFeetToMeters;
    end.lowNumberedEnd = lowNumberedEnd;
    return end;
}

} // namespace

bool RunwayDatabase::Load(const std::filesystem::path& path, std::string& error) {
    error.clear();
    m_loaded = false;
    m_runwayEnds.clear();
    m_airportRunwayEnds.clear();

    std::ifstream in(path);
    if (!in) {
        error = "Failed to open runway database: " + path.string();
        return false;
    }

    std::string line;
    if (!std::getline(in, line)) {
        error = "Runway database is empty: " + path.string();
        return false;
    }

    const std::vector<std::string> header = ParseCsvLine(line);
    std::unordered_map<std::string, size_t> columns;
    for (size_t i = 0; i < header.size(); ++i) {
        columns.emplace(Trim(header[i]), i);
    }

    const std::array<const char*, 9> requiredColumns = {
        "airport_ident",
        "closed",
        "le_ident",
        "le_latitude_deg",
        "le_longitude_deg",
        "he_ident",
        "he_latitude_deg",
        "he_longitude_deg",
        "length_ft",
    };
    for (const char* column : requiredColumns) {
        if (!columns.contains(column)) {
            error = std::string("Runway database missing required column: ") + column;
            return false;
        }
    }

    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = ParseCsvLine(line);
        const int closed = static_cast<int>(ParseDoubleOr(fields, columns, "closed", 0.0));
        if (closed != 0) {
            continue;
        }

        if (!HasDouble(fields, columns, "le_latitude_deg") || !HasDouble(fields, columns, "le_longitude_deg") ||
            !HasDouble(fields, columns, "he_latitude_deg") || !HasDouble(fields, columns, "he_longitude_deg")) {
            continue;
        }

        const std::string airportIdent = ToUpperAscii(ParseStringOr(fields, columns, "airport_ident"));
        const std::string leIdent = ParseStringOr(fields, columns, "le_ident");
        const std::string heIdent = ParseStringOr(fields, columns, "he_ident");
        if (airportIdent.empty() || leIdent.empty() || heIdent.empty()) {
            continue;
        }

        const double leLat = ParseDoubleOr(fields, columns, "le_latitude_deg", 0.0);
        const double leLon = ParseDoubleOr(fields, columns, "le_longitude_deg", 0.0);
        const double heLat = ParseDoubleOr(fields, columns, "he_latitude_deg", 0.0);
        const double heLon = ParseDoubleOr(fields, columns, "he_longitude_deg", 0.0);
        if (!IsValidLatLon(leLat, leLon) || !IsValidLatLon(heLat, heLon)) {
            continue;
        }

        const double lengthFeet = ParseDoubleOr(fields, columns, "length_ft", 0.0);
        if (lengthFeet <= 0.0) {
            continue;
        }

        const std::string surface = ParseStringOr(fields, columns, "surface");
        const double widthFeet = ParseDoubleOr(fields, columns, "width_ft", 0.0);
        const double leElevationFeet = ParseDoubleOr(fields, columns, "le_elevation_ft", ParseDoubleOr(fields, columns, "he_elevation_ft", 0.0));
        const double heElevationFeet = ParseDoubleOr(fields, columns, "he_elevation_ft", leElevationFeet);
        const double leHeading = BearingDegBetween(leLat, leLon, heLat, heLon);
        const double heHeading = BearingDegBetween(heLat, heLon, leLat, leLon);

        const int leIndex = static_cast<int>(m_runwayEnds.size());
        m_runwayEnds.push_back(
            MakeRunwayEnd(airportIdent, leIdent, heIdent, surface, leLat, leLon, leElevationFeet, leHeading, lengthFeet, widthFeet, true));
        m_airportRunwayEnds[airportIdent].push_back(leIndex);

        const int heIndex = static_cast<int>(m_runwayEnds.size());
        m_runwayEnds.push_back(
            MakeRunwayEnd(airportIdent, heIdent, leIdent, surface, heLat, heLon, heElevationFeet, heHeading, lengthFeet, widthFeet, false));
        m_airportRunwayEnds[airportIdent].push_back(heIndex);
    }

    for (auto& [ident, indices] : m_airportRunwayEnds) {
        std::sort(indices.begin(), indices.end(), [this](int a, int b) {
            const RunwayEndRecord& lhs = m_runwayEnds[static_cast<size_t>(a)];
            const RunwayEndRecord& rhs = m_runwayEnds[static_cast<size_t>(b)];
            if (std::abs(lhs.lengthMeters - rhs.lengthMeters) > 0.1) {
                return lhs.lengthMeters > rhs.lengthMeters;
            }
            if (lhs.lowNumberedEnd != rhs.lowNumberedEnd) {
                return lhs.lowNumberedEnd;
            }
            return lhs.runwayIdent < rhs.runwayIdent;
        });
    }

    m_loaded = !m_runwayEnds.empty();
    if (!m_loaded) {
        error = "Runway database loaded no usable runway ends: " + path.string();
        return false;
    }
    return true;
}

std::string RunwayDatabase::ResolveAirportIdent(const std::string& ident) const {
    const std::string normalized = ToUpperAscii(Trim(ident));
    if (normalized.empty()) {
        return {};
    }
    if (m_airportRunwayEnds.contains(normalized)) {
        return normalized;
    }
    if (normalized.size() == 3) {
        const std::array<std::string, 3> candidates = {
            "K" + normalized,
            "P" + normalized,
            "C" + normalized,
        };
        for (const std::string& candidate : candidates) {
            if (m_airportRunwayEnds.contains(candidate)) {
                return candidate;
            }
        }
    }
    return {};
}

const std::vector<int>& RunwayDatabase::RunwayEndIndicesForAirport(const std::string& ident) const {
    const std::string resolved = ResolveAirportIdent(ident);
    if (resolved.empty()) {
        return kEmptyIndices;
    }
    const auto it = m_airportRunwayEnds.find(resolved);
    return it != m_airportRunwayEnds.end() ? it->second : kEmptyIndices;
}

const RunwayEndRecord* RunwayDatabase::GetRunwayEnd(int runwayEndIndex) const {
    if (runwayEndIndex < 0 || static_cast<size_t>(runwayEndIndex) >= m_runwayEnds.size()) {
        return nullptr;
    }
    return &m_runwayEnds[static_cast<size_t>(runwayEndIndex)];
}

int RunwayDatabase::DefaultRunwayEndIndexForAirport(const std::string& ident) const {
    const std::vector<int>& indices = RunwayEndIndicesForAirport(ident);
    if (indices.empty()) {
        return -1;
    }
    return indices.front();
}
