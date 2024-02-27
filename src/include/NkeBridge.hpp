#pragma once

// OK KISS for once
// TODO do coversions from kelvi and nots to ms here ?
// leave that to nmea library that has nice support for it.!!

// TODO we try to handle all the data conversions here.. maybe its a bad idea maybe its a good idea
// //This is a pure math thing so unit tests can be made!
#include <cmath>
#include <cstdint>

#include "NkeChannel.hpp"
using Nke::Channel;

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

class Xte : public NkeN2kData
{
public:
    void setN2k(const double &n2k) override
    {
        int16_t tmp = static_cast<int16_t>(n2k / 18.52); // compute to 100*nautical miles
        int16_t tmp2 = 1000 + tmp;
        if (tmp2 < 0)
        {
            tmp2 = 0;
        }
        if (tmp2 > 2000)
        {
            tmp2 = 2000;
        }
        m_val_nke = tmp2;
        m_val_n2k = n2k;
    }
    void setNke(const uint16_t &nke) override
    {
        // convert to +-
        int16_t tmp = nke - 1000;

        m_val_n2k = tmp * 18.52;
        m_val_nke = nke;
    }
};

class Dtw : public NkeN2kData
{
public:
    // distance to waypoint in integer value meters
    void setN2k(const double &n2k) override
    {
        m_val_nke = static_cast<uint8_t>(n2k / 185.2); // (meters /1852)*10 ..presented in 0.1 steps
        m_val_n2k = n2k;
    }
    // distance to waypoint integer value in nautical miles div 10
    void setNke(const uint16_t &nke) override
    {
        m_val_n2k = nke * 185.2;
        m_val_nke = nke;
    }
};

class SOG : public NkeN2kData
{
public:
    void setN2k(const double &n2k) override
    {
        // sog is in m/s for n2k..
        // sog is m/s * 100 for n2k.
        // Serial.printf("SetSog to %f\n", n2k);
        m_val_n2k = n2k;
        // m_val_nke = (n2k * 500.0);
        // nke is 190 per knot meaning its 1900m/h / 190 => 10m/h steps
        // each step is 0.000277778 m/s
        // is it 190 or 185.2 :manshrugging:
        m_val_nke = (n2k * 3.6 * 190);

        //  m_val_nke = static_cast<uint16_t>(n2k * (180 / M_PI)); // uint16_t(DegToRad(n2k)); // this need to be verified at some point
        //  m_val_n2k = n2k;
    }
    void setNke(const uint16_t &nke) override
    {
        // m_val_n2k = nke * (M_PI / 180.0);
        m_val_n2k = (nke / 3.6 * 190);
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

// position is made from 4 channels

// pgns
//  case 129026UL: HandleCOGSOG(N2kMsg);
//  case 129029UL: HandleGNSS(N2kMsg);
//  case 129025UL: HandlePosition(N2kMsg);

// void tN2kDataToNMEA0183::HandlePosition(const tN2kMsg &N2kMsg) {
/*
  if ( ParseN2kPGN129025(N2kMsg, Latitude, Longitude) ) {
    LastPositionTime=millis();
  }
}

void tN2kDataToNMEA0183::HandleCOGSOG(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kHeadingReference HeadingReference;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,COG,SOG) ) {
    LastCOGSOGTime=millis();
    double MCOG=( !N2kIsNA(COG) && !N2kIsNA(Variation)?COG-Variation:NMEA0183DoubleNA );
    if ( HeadingReference==N2khr_magnetic ) {
      MCOG=COG;
      if ( !N2kIsNA(Variation) ) COG-=Variation;
    }
    if ( NMEA0183SetVTG(NMEA0183Msg,COG,MCOG,SOG) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

  double Latitude;
  double Longitude;
  double Altitude;
  double Variation;
  double Heading;
  double COG;
  double SOG;

*/
class Position
{
public:
    Position(uint8_t lat_hi = (uint8_t)Channel::POS_LAT_HR_MIN,
             uint8_t lat_low = (uint8_t)Channel::POS_LAT_SECONDS,
             uint8_t long_hi = (uint8_t)Channel::POS_LONG_HR_MIN,
             uint8_t long_low = (uint8_t)Channel::POS_LONG_SECONS)
        : m_chan_lat_hi(lat_hi), m_chan_lat_low(lat_low), m_chan_long_hi(long_hi), m_chan_long_low(long_low)
    {
    }
    // maybe we can make it generic enough?
    void setNkePos(uint16_t val, uint8_t chan)
    {
        if (chan == m_chan_lat_hi || chan == m_chan_long_hi)
        {
            tmp_hi = val;
        }
        if (chan == m_chan_lat_low)
        {
            // change both at the same time.
            m_lat_hi_nke = tmp_hi;
            m_lat_low_nke = val;
            nkeToDouble(m_lat_hi_nke, m_lat_low_nke, m_latitude);
        }
        if (chan == m_chan_long_low)
        {
            // change both at the same time.
            m_long_hi_nke = tmp_hi;
            m_long_low_nke = val;
            nkeToDouble(m_long_hi_nke, m_long_low_nke, m_longditude);
        }
        /// need to understand the format of the PGN
    }
    // void dmsToDmm()
    void nkeToDouble(uint16_t hi, uint16_t low, double &val)
    {
        uint8_t unknown = hi >> 14;
        double hour = (hi >> 6) & 0xFF; // whats the last bit for ? maybe 4 is east an 8 is south ?
        double min = (hi & 0x3F);
        // 0-9999 is min_fraction values
        double min_fraction = low / 1000.0;       // shift 10 decimal places
        val = hour + ((min + min_fraction) / 60); // convert to FP
    }
    //  consider using a 32 bit
    void doubleToNke(double val, uint16_t &hi, uint16_t &low)
    {
        uint16_t hour = val;
        val -= hour;
        val = val * 60;
        uint16_t minutes = val;
        val -= minutes;
        // DMM in 1/1000
        low = val * 1000;

        // val = val * 60;
        // uint16_t seconds = val;
    }
    // To degree decimal minutes format
    void setN2kPos(double latitude, double longditude)
    {
        uint16_t temp1, temp2;
        doubleToNke(latitude, temp1, temp2);
        m_lat_hi_nke = temp1;
        m_lat_low_nke = temp2;

        doubleToNke(longditude, temp1, temp2);
        m_long_hi_nke = temp1;
        m_long_low_nke = temp2;
        m_latitude = latitude;
        m_longditude = longditude;
    }

private:
    volatile uint16_t m_lat_hi_nke = 0;
    volatile uint16_t m_lat_low_nke = 0;
    volatile uint16_t m_long_hi_nke = 0;
    volatile uint16_t m_long_low_nke = 0;
    uint16_t tmp_hi = 0;
    double m_latitude;
    double m_longditude;

    uint8_t m_chan_lat_hi;
    uint8_t m_chan_lat_low;
    uint8_t m_chan_long_hi;
    uint8_t m_chan_long_low;
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

    const SOG &speedOverGround() { return m_speedOverGround; }

    void setSpeedOverGround(double sog) { m_speedOverGround.setN2k(sog); }
    void setSpeedOverGround(uint16_t sog) { m_speedOverGround.setNke(sog); }

    const NkeN2kAngle &courseOverGround() { return m_courseOverGround; }
    void setCourseOverGround(double cog) { m_courseOverGround.setN2k(cog); }
    void setCourseOverGround(uint16_t cog) { m_courseOverGround.setNke(cog); }

    const Position &position() { return m_position; }
    void setPosition(double lat, double lng) { m_position.setN2kPos(lat, lng); }
    void setPosition(uint16_t val, uint8_t chan) { m_position.setNkePos(val, chan); }

    // Off track to waypoint
    const Xte &xte() { return m_xte; }
    void setXte(double val) { m_xte.setN2k(val); }
    void setXte(uint16_t val) { m_xte.setNke(val); }

    // Bearing to waypoint
    const NkeN2kAngle &btw() { return m_btw; }
    void setBtw(double val) { m_btw.setN2k(val); }
    void setBtw(uint16_t val) { m_btw.setNke(val); }

    // Distance to waypoint
    const Dtw &dtw() { return m_dtw; }
    void setDtw(double val) { m_dtw.setN2k(val); }
    void setDtw(uint16_t val) { m_dtw.setNke(val); }

private:
    double m_boatSpeed = 0.0; // knots
    SOG m_speedOverGround;
    Xte m_xte;
    NkeN2kAngle m_btw;
    Dtw m_dtw;
    // double m_speedOverGround = 0.0; // knots
    double m_windSpeed = 0.0; // knots
    NkeN2kAngle m_windAngle;
    // double m_windAngle = 0.0;       // radians ?
    RudderAngle m_rudderAngle;

    // double m_rudderAngle = 0.0; // radians ?
    double m_waterdepth = 0.0; // meters
    double m_waterTemp = 0.0;  // celcius
    // NkeN2kAngle m_windAngle;
    Position m_position;
    NkeN2kAngle m_heading;
    NkeN2kAngle m_courseOverGround;

    // double m_heading = 0.0; // radians?
};