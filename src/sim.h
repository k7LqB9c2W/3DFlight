#pragma once

#include <algorithm>

namespace flight {

struct Double3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Double3 operator+(const Double3& a, const Double3& b);
Double3 operator-(const Double3& a, const Double3& b);
Double3 operator*(const Double3& v, double s);
Double3 operator/(const Double3& v, double s);

double Dot(const Double3& a, const Double3& b);
Double3 Cross(const Double3& a, const Double3& b);
double Length(const Double3& v);
Double3 Normalize(const Double3& v);

double DegToRad(double degrees);
double RadToDeg(double radians);

constexpr double kEarthRadiusMeters = 6371000.0;

Double3 GeodeticToEcef(double latitudeRad, double longitudeRad, double altitudeMeters);
void BuildEnuBasis(double latitudeRad, double longitudeRad, Double3& east, Double3& north, Double3& up);

struct FlightStart {
    double latitudeDeg = 37.6188056;
    double longitudeDeg = -122.3754167;
    double altitudeMeters = 9144.0;
    double speedMps = 250.0;
    double headingDeg = 90.0;
};

struct InputState {
    bool pitchUp = false;
    bool pitchDown = false;
    bool rollLeft = false;
    bool rollRight = false;
    bool yawLeft = false;
    bool yawRight = false;
    bool throttleUp = false;
    bool throttleDown = false;
    bool altitudeUp = false;
    bool altitudeDown = false;
    bool resetPressed = false;
};

class FlightSim {
public:
    FlightSim();

    void SetStart(const FlightStart& start);
    [[nodiscard]] const FlightStart& GetStart() const { return m_start; }

    void Respawn();
    void Update(double dtSeconds, const InputState& input, double timeScale);

    [[nodiscard]] double LatitudeRad() const { return m_latitudeRad; }
    [[nodiscard]] double LongitudeRad() const { return m_longitudeRad; }
    [[nodiscard]] double AltitudeMeters() const { return m_altitudeMeters; }
    [[nodiscard]] double SpeedMps() const { return m_speedMps; }
    [[nodiscard]] double HeadingRad() const { return m_headingRad; }
    [[nodiscard]] double PitchRad() const { return m_pitchRad; }
    [[nodiscard]] double RollRad() const { return m_rollRad; }

    [[nodiscard]] double LatitudeDeg() const { return RadToDeg(m_latitudeRad); }
    [[nodiscard]] double LongitudeDeg() const { return RadToDeg(m_longitudeRad); }
    [[nodiscard]] double HeadingDeg() const { return RadToDeg(m_headingRad); }

    [[nodiscard]] Double3 PlaneEcef() const;
    [[nodiscard]] Double3 ForwardEcef() const;
    [[nodiscard]] Double3 UpEcef() const;

private:
    FlightStart m_start;
    double m_latitudeRad = 0.0;
    double m_longitudeRad = 0.0;
    double m_altitudeMeters = 0.0;
    double m_speedMps = 0.0;
    double m_headingRad = 0.0;
    double m_pitchRad = 0.0;
    double m_rollRad = 0.0;
};

} // namespace flight
