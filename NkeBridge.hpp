#pragma once

// OK KISS for once
// TODO do coversions from kelvi and nots to ms here ?
// leave that to nmea library that has nice support for it.!!

// TODO we try to handle all the data conversions here.. maybe its a bad idea maybe its a good idea
// //This is a pure math thing so unit tests can be made!
#include <cmath>

class NkeN2kData
{
public:
    volatile const uint16_t &Nke() const { return m_val_nke; }
    volatile const double &N2k() const { return m_val_n2k; }

    virtual void setN2k(const double &n2k)
    {
        m_val_nke = uint16_t(n2k);
        m_val_n2k = m_val_n2k;
    }
    virtual void setNke(const uint16_t &nke)
    {
        m_val_n2k = double(nke);
        m_val_nke = nke;
    }

    // this makes it easier but need to look how it looks below!
    /* template<typename T>
     void set(const T &val)
     {

     }*/

protected:
    volatile double m_val_n2k = 0.0;
    volatile uint16_t m_val_nke = 0;
};
// so the above is ofc total bs as no such conversion exists.. however.. !!

// Think its applicable for wind / rudder / heading / trim / pitch
class NkeN2kAngle : public NkeN2kData
{
public:
    void setN2k(const double &n2k) override
    {
        m_val_nke = static_cast<uint16_t>(n2k * (180 / M_PI)); // uint16_t(DegToRad(n2k)); // this need to be verified at some point
        m_val_n2k = n2k;
    }
    void setNke(const uint16_t &nke) override
    {
        m_val_n2k = nke * (M_PI / 180.0);
        m_val_nke = nke;
    }
    // RadToDeg(m_bridge.windAngle())
};

class RudderAngle : public NkeN2kData
{
public:
    void setN2k(const double &n2k) override
    {
        // if negative angle..  (need to read n2k doc)
        uint16_t m_sign = 0;
        if (n2k < 0.0)
        {
            m_sign = (0x4 << 8); // maybe its 12 bit :manshrug
            // m_val_nke|= static_cast<uint16_t>(n2k * (180 / M_PI));
        }
        m_val_nke = m_sign | static_cast<uint16_t>(n2k * (180 / M_PI)); // uint16_t(DegToRad(n2k)); // this need to be verified at some point
        m_val_n2k = n2k;
    }
    void setNke(const uint16_t &nke) override
    {
        m_val_nke = nke;

        // int computed = (nke / 3);
        // if (nke > (180 * 3))
        // {
        // computed = -(360 - computed);
        m_val_n2k = nke * (M_PI / (180.0 * 3));

        // Serial.printf("Rudder Angle %d\n",computed);
        /*}
        else
        {
            m_val_n2k = computed * (M_PI / (180.0 * 3));
        }*/
        // we store the computed value or we compute in Autopilot?
    }

    uint16_t getAsAngle() // needed by AP
    {
        return (m_val_nke / 3);
        // return computed
    }
};

class NkeBridge
{
public:
    // NkeBridge(Do we need this probably not);
    // reference isnt great maybe just do the copy asuming we have enough cycles
    double boatSpeed() { return m_boatSpeed; }
    void setBoatSpeed(double speed) { m_boatSpeed = speed; }

    // this is the easiest approach
    const NkeN2kAngle &windAngle() { return m_windAngle; }
    void setWindAngle(double angle) { m_windAngle.setN2k(angle); }
    void setWindAngle(uint16_t angle) { m_windAngle.setNke(angle); }

    double windSpeed() { return m_windSpeed; }
    void setWindSpeed(double speed) { m_windSpeed = speed; }

    double waterDepth() { return m_waterdepth; }
    void setWaterDepth(double depth) { m_waterdepth = depth; }

    double waterTemp() { return m_waterTemp; }
    void setWaterTemp(double temp) { m_waterTemp = temp; }

    const NkeN2kData &rudderAngle() { return m_rudderAngle; }
    // todo use templates and get a single set function?
    //  double rudderAngle() { return m_rudderAngle.; }
    //  uint16_t getRudderAngle() { return m_rudderAngle; }
    void setRudderAngle(double angle) { m_rudderAngle.setN2k(angle); }
    void setRunnerAngle(uint16_t angle) { m_rudderAngle.setNke(angle); }

    const NkeN2kAngle &heading() { return m_heading; }
    // double heading() { return m_heading; }
    void setHeading(double heading) { m_heading.setN2k(heading); }
    void setHeading(uint16_t heading) { m_heading.setNke(heading); }

    double speedOverGround() { return m_speedOverGround; }
    void setSpeedOverGround(double sog) { m_speedOverGround = sog; }

private:
    double m_boatSpeed = 0.0;       // knots
    double m_speedOverGround = 0.0; // knots
    double m_windSpeed = 0.0;       // knots
    NkeN2kAngle m_windAngle;
    // double m_windAngle = 0.0;       // radians ?
    RudderAngle m_rudderAngle;

    // double m_rudderAngle = 0.0; // radians ?
    double m_waterdepth = 0.0; // meters
    double m_waterTemp = 0.0;  // celcius
    // NkeN2kAngle m_windAngle;

    NkeN2kAngle m_heading;

    // double m_heading = 0.0; // radians?
};