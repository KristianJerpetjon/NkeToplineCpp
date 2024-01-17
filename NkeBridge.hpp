#pragma once

// OK KISS for once
// TODO do coversions from kelvi and nots to ms here ?
// leave that to nmea library that has nice support for it.!!
class NkeBridge
{
public:
    // NkeBridge(Do we need this probably not);
    // reference isnt great maybe just do the copy asuming we have enough cycles
    double boatSpeed() { return m_boatSpeed; }
    void setBoatSpeed(double speed) { m_boatSpeed = speed; }

    double windAngle() { return m_windAngle; }
    void setWindAngle(double angle) { m_windAngle = angle; }

    double windSpeed() { return m_windSpeed; }
    void setWindSpeed(double speed) { m_windSpeed = speed; }

    double waterDepth() { return m_waterdepth; }
    void setWaterDepth(double depth) { m_waterdepth = depth; }

    double waterTemp() { return m_waterTemp; }
    void setWaterTemp(double temp) { m_waterTemp = temp; }

    double rudderAngle() { return m_rudderAngle; }
    void setRudderAngle(double angle) { m_rudderAngle = angle; }

    double heading() { return m_heading; }
    void setHeading(double heading) { m_heading = heading; }

    double speedOverGround() { return m_speedOverGround; }
    void setSpeedOverGround(double sog) { m_speedOverGround = sog; }

private:
    double m_boatSpeed = 0.0;       // knots
    double m_speedOverGround = 0.0; // knots
    double m_windSpeed = 0.0;       // knots
    double m_windAngle = 0.0;       // radians ?
    double m_rudderAngle = 0.0;     // radians ?
    double m_waterdepth = 0.0;      // meters
    double m_waterTemp = 0.0;       // celcius
    double m_heading = 0.0;         // radians?
};