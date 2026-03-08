#include "sim.h"

#include <cmath>

namespace flight {

namespace {

constexpr double kPi = 3.14159265358979323846;

double WrapRadiansPi(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

void WrapLongitude(double& lon) {
    lon = WrapRadiansPi(lon);
}

} // namespace

Double3 operator+(const Double3& a, const Double3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Double3 operator-(const Double3& a, const Double3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Double3 operator*(const Double3& v, double s) {
    return {v.x * s, v.y * s, v.z * s};
}

Double3 operator/(const Double3& v, double s) {
    return {v.x / s, v.y / s, v.z / s};
}

double Dot(const Double3& a, const Double3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Double3 Cross(const Double3& a, const Double3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

double Length(const Double3& v) {
    return std::sqrt(Dot(v, v));
}

Double3 Normalize(const Double3& v) {
    const double len = Length(v);
    if (len < 1e-12) {
        return {0.0, 0.0, 0.0};
    }
    return v / len;
}

double DegToRad(double degrees) {
    return degrees * (kPi / 180.0);
}

double RadToDeg(double radians) {
    return radians * (180.0 / kPi);
}

Double3 GeodeticToEcef(double latitudeRad, double longitudeRad, double altitudeMeters) {
    const double r = kEarthRadiusMeters + altitudeMeters;
    const double cosLat = std::cos(latitudeRad);
    const double sinLat = std::sin(latitudeRad);
    const double cosLon = std::cos(longitudeRad);
    const double sinLon = std::sin(longitudeRad);

    return {
        r * cosLat * cosLon,
        r * sinLat,
        r * cosLat * sinLon,
    };
}

void BuildEnuBasis(double latitudeRad, double longitudeRad, Double3& east, Double3& north, Double3& up) {
    const double cosLat = std::cos(latitudeRad);
    const double sinLat = std::sin(latitudeRad);
    const double cosLon = std::cos(longitudeRad);
    const double sinLon = std::sin(longitudeRad);

    east = {-sinLon, 0.0, cosLon};
    north = {-sinLat * cosLon, cosLat, -sinLat * sinLon};
    up = {cosLat * cosLon, sinLat, cosLat * sinLon};
}

FlightSim::FlightSim() {
    Respawn();
}

void FlightSim::SetStart(const FlightStart& start) {
    m_start = start;
}

void FlightSim::Respawn() {
    m_latitudeRad = DegToRad(m_start.latitudeDeg);
    m_longitudeRad = DegToRad(m_start.longitudeDeg);
    m_altitudeMeters = m_start.altitudeMeters;
    m_speedMps = m_start.speedMps;
    m_headingRad = DegToRad(m_start.headingDeg);
    m_pitchRad = 0.0;
    m_rollRad = 0.0;
    m_altitudeHoldEnabled = false;
    m_altitudeHoldTargetMeters = m_altitudeMeters;
    m_headingHoldEnabled = false;
    m_headingHoldTargetRad = m_headingRad;
    WrapLongitude(m_longitudeRad);
}

void FlightSim::Update(double dtSeconds, const InputState& input, double timeScale) {
    if (input.resetPressed) {
        Respawn();
        return;
    }

    const double dt = dtSeconds * std::clamp(timeScale, 0.01, 100.0);

    const double pitchInput = (input.pitchUp ? 1.0 : 0.0) - (input.pitchDown ? 1.0 : 0.0);
    const double rollInput = (input.rollRight ? 1.0 : 0.0) - (input.rollLeft ? 1.0 : 0.0);
    const double yawInput = (input.yawRight ? 1.0 : 0.0) - (input.yawLeft ? 1.0 : 0.0);
    const double throttleInput = (input.throttleUp ? 1.0 : 0.0) - (input.throttleDown ? 1.0 : 0.0);
    const double climbInput = (input.altitudeUp ? 1.0 : 0.0) - (input.altitudeDown ? 1.0 : 0.0);

    constexpr double kPitchRate = 0.45;
    constexpr double kRollRate = 0.80;
    constexpr double kYawRate = 0.70;
    constexpr double kThrottleAccel = 40.0;
    constexpr double kManualClimbRate = 120.0;
    constexpr double kGravity = 9.80665;
    constexpr double kHeadingHoldRate = 1.10;
    constexpr double kAltitudeHoldGain = 0.45;
    constexpr double kAltitudeHoldMaxVerticalSpeed = 120.0;
    constexpr double kAltitudeHoldMaxAssistRate = 220.0;

    m_pitchRad += pitchInput * kPitchRate * dt;
    m_rollRad += rollInput * kRollRate * dt;
    m_headingRad += yawInput * kYawRate * dt;
    if (m_headingHoldEnabled && std::abs(yawInput) > 1e-6) {
        m_headingHoldTargetRad += yawInput * kYawRate * dt;
        WrapLongitude(m_headingHoldTargetRad);
    }

    m_pitchRad = std::clamp(m_pitchRad, DegToRad(-40.0), DegToRad(40.0));
    m_rollRad = std::clamp(m_rollRad, DegToRad(-75.0), DegToRad(75.0));

    m_speedMps += throttleInput * kThrottleAccel * dt;
    m_speedMps = std::clamp(m_speedMps, 60.0, 650.0);

    const double horizontalSpeed = m_speedMps * std::cos(m_pitchRad);
    // Coordinated-turn approximation: bank angle induces heading change.
    const double speedForTurn = std::max(45.0, std::abs(horizontalSpeed));
    double bankTurnRate = (kGravity * std::tan(m_rollRad)) / speedForTurn;
    bankTurnRate = std::clamp(bankTurnRate, DegToRad(-30.0), DegToRad(30.0));
    m_headingRad += bankTurnRate * dt;
    WrapLongitude(m_headingRad);

    if (m_headingHoldEnabled) {
        const bool steeringInputActive = (std::abs(yawInput) > 1e-6) || (std::abs(rollInput) > 1e-6);
        if (steeringInputActive) {
            m_headingHoldTargetRad = m_headingRad;
        } else {
            const double headingError = WrapRadiansPi(m_headingHoldTargetRad - m_headingRad);
            m_headingRad += std::clamp(headingError, -kHeadingHoldRate * dt, kHeadingHoldRate * dt);
            WrapLongitude(m_headingRad);
        }
    }

    const double verticalFromPitch = m_speedMps * std::sin(m_pitchRad);
    double climbRateCommand = climbInput * kManualClimbRate;
    if (m_altitudeHoldEnabled) {
        if (std::abs(climbInput) > 1e-6) {
            m_altitudeHoldTargetMeters = std::max(10.0, m_altitudeHoldTargetMeters + climbInput * kManualClimbRate * dt);
        }
        const double altitudeError = m_altitudeHoldTargetMeters - m_altitudeMeters;
        const double targetVerticalSpeed = std::clamp(
            altitudeError * kAltitudeHoldGain,
            -kAltitudeHoldMaxVerticalSpeed,
            kAltitudeHoldMaxVerticalSpeed);
        climbRateCommand = std::clamp(
            targetVerticalSpeed - verticalFromPitch,
            -kAltitudeHoldMaxAssistRate,
            kAltitudeHoldMaxAssistRate);
    }

    const double dNorth = std::cos(m_headingRad) * horizontalSpeed * dt;
    const double dEast = std::sin(m_headingRad) * horizontalSpeed * dt;
    const double dUp = (verticalFromPitch + climbRateCommand) * dt;

    m_altitudeMeters = std::max(10.0, m_altitudeMeters + dUp);
    if (m_altitudeHoldEnabled) {
        m_altitudeHoldTargetMeters = std::max(10.0, m_altitudeHoldTargetMeters);
    }

    const double r = kEarthRadiusMeters + m_altitudeMeters;
    m_latitudeRad += dNorth / r;
    m_latitudeRad = std::clamp(m_latitudeRad, DegToRad(-89.9), DegToRad(89.9));

    const double cosLat = std::max(0.01, std::cos(m_latitudeRad));
    m_longitudeRad += dEast / (r * cosLat);
    WrapLongitude(m_longitudeRad);
}

void FlightSim::SetKinematicStateRadians(
    double latitudeRad,
    double longitudeRad,
    double altitudeMeters,
    double speedMps,
    double headingRad,
    double pitchRad,
    double rollRad) {
    m_latitudeRad = std::clamp(latitudeRad, DegToRad(-89.9), DegToRad(89.9));
    m_longitudeRad = longitudeRad;
    WrapLongitude(m_longitudeRad);
    m_altitudeMeters = std::max(0.0, altitudeMeters);
    m_speedMps = std::max(0.0, speedMps);
    m_headingRad = headingRad;
    WrapLongitude(m_headingRad);
    m_pitchRad = std::clamp(pitchRad, DegToRad(-89.0), DegToRad(89.0));
    m_rollRad = std::clamp(rollRad, DegToRad(-89.0), DegToRad(89.0));
    if (m_altitudeHoldEnabled) {
        m_altitudeHoldTargetMeters = m_altitudeMeters;
    }
    if (m_headingHoldEnabled) {
        m_headingHoldTargetRad = m_headingRad;
    }
}

void FlightSim::EnableAltitudeHoldAtCurrentAltitude() {
    m_altitudeHoldEnabled = true;
    m_altitudeHoldTargetMeters = std::max(10.0, m_altitudeMeters);
}

void FlightSim::DisableAltitudeHold() {
    m_altitudeHoldEnabled = false;
    m_altitudeHoldTargetMeters = std::max(10.0, m_altitudeMeters);
}

void FlightSim::EnableHeadingHoldAtCurrentHeading() {
    m_headingHoldEnabled = true;
    m_headingHoldTargetRad = m_headingRad;
    WrapLongitude(m_headingHoldTargetRad);
}

void FlightSim::DisableHeadingHold() {
    m_headingHoldEnabled = false;
    m_headingHoldTargetRad = m_headingRad;
    WrapLongitude(m_headingHoldTargetRad);
}

void FlightSim::ResetPitch() {
    m_pitchRad = 0.0;
}

void FlightSim::ResetRoll() {
    m_rollRad = 0.0;
}

void FlightSim::StabilizeFlight() {
    EnableAltitudeHoldAtCurrentAltitude();
    EnableHeadingHoldAtCurrentHeading();
    ResetPitch();
    ResetRoll();
}

Double3 FlightSim::PlaneEcef() const {
    return GeodeticToEcef(m_latitudeRad, m_longitudeRad, m_altitudeMeters);
}

Double3 FlightSim::ForwardEcef() const {
    Double3 east, north, up;
    BuildEnuBasis(m_latitudeRad, m_longitudeRad, east, north, up);

    const double eastComp = std::sin(m_headingRad) * std::cos(m_pitchRad);
    const double northComp = std::cos(m_headingRad) * std::cos(m_pitchRad);
    const double upComp = std::sin(m_pitchRad);

    return Normalize(east * eastComp + north * northComp + up * upComp);
}

Double3 FlightSim::UpEcef() const {
    Double3 east, north, up;
    BuildEnuBasis(m_latitudeRad, m_longitudeRad, east, north, up);
    return Normalize(up);
}

} // namespace flight
