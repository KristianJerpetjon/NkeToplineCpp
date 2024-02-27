#pragma once

#include <N2kMessages.h>
#include "NkeBridge.hpp"

// we can do these as a bunch of lambdas.. is that ugly ?

// stick data received on either Nke or N2k into the NkeBridge
class N2kNkeHandler
{
public:
    N2kNkeHandler(NkeBridge &bridge) : m_bridge(bridge) {}
    virtual void handleN2kData(const tN2kMsg &msg) {}
    virtual void handleNkeData(const tNkeMsg &msg) {}

protected:
    NkeBridge &m_bridge;
};

class HandleWaterTemp : public N2kNkeHandler
{
public:
    HandleWaterTemp(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
        unsigned char SID;
        double WaterTemperature;
        double OutsideAmbientAirTemperature;
        double AtmosphericPressure;
        // Parse

        /* does not support 12 as they are deprecated
    130310..............Environmental Parameters (Water Temperature)
    130311..............Environmental Parameters (Water Temperature) Not implemented
    //130312..............Environmental Parameters (Water Temperature) Not implemented
    130316..............Temperature, Extended Range
        */

        tN2kTempSource tempSource;
        tN2kHumiditySource humiditySource;
        double humidity;
        uint8_t instance;
        // double ActualTemperature;
        double SetTemperature;
        switch (N2kMsg.PGN)
        {
        case 130310L:
            if (!ParseN2kOutsideEnvironmentalParameters(N2kMsg, SID, WaterTemperature, OutsideAmbientAirTemperature, AtmosphericPressure))
            {
                return;
            }
            tempSource = tN2kTempSource::N2kts_SeaTemperature;
            break;
        case 130311L:

            /*const tN2kMsg &N2kMsg, unsigned char &SID, tN2kTempSource &TempSource, double &Temperature,
                           tN2kHumiditySource &HumiditySource, double &Humidity, double &AtmosphericPressure*/
            if (!ParseN2kEnvironmentalParameters(N2kMsg, SID, tempSource, WaterTemperature, humiditySource, humidity, AtmosphericPressure))
            {
                return;
            }
            break;
        case 130316L:
            if (!ParseN2kTemperatureExt(N2kMsg, SID, instance, tempSource, WaterTemperature, SetTemperature))
            {
                return;
            }

            break;
        default:
            return;
            /*PrintLabelValWithConversionCheckUnDef("Water temp: ",WaterTemperature,&KelvinToC);
            PrintLabelValWithConversionCheckUnDef(", outside ambient temp: ",OutsideAmbientAirTemperature,&KelvinToC);
            PrintLabelValWithConversionCheckUnDef(", pressure: ",AtmosphericPressure,0,true);*/
        }
        if (tempSource != tN2kTempSource::N2kts_SeaTemperature)
        {
            return;
        }
        // todo deal with all the rest of the data here
        if (!N2kIsNA(WaterTemperature))
        {
            // maybe right maybe wrong
            m_bridge.setWaterTemp(KelvinToC(WaterTemperature));

            // setData(uint16_t(KelvinToC(WaterTemperature) / stepsize));
        }
    }

    void handleNkeData(const tNkeMsg &msg) override {}
};

class HandleBoatSpeed : public N2kNkeHandler
{
public:
    HandleBoatSpeed(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
        unsigned char SID;
        double SOW;
        double SOG;
        tN2kSpeedWaterReferenceType SWRT;
        // if cant parse we exit
        if (ParseN2kBoatSpeed(N2kMsg, SID, SOW, SOG, SWRT))
        {
            if (!N2kIsNA(SOW))
            {
                m_bridge.setBoatSpeed(SOW);
                // setData(0);
            }
            else
            {

                // We do the math in knots ..
                //    setData(uint16_t(msToKnots(SOW) / stepsize));
                //    m_fast_data = m_data;
            }
            if (!N2kIsNA(SOG))
            {
                m_bridge.setSpeedOverGround(SOG);
            }
        }
    }
    void handleNkeData(const tNkeMsg &msg) override {}
};

/*
TODO!!
class HandleTemp : public N2kNkeHandler
{
public:
    void handleN2kData(const tN2kMsg &msg) override
    {
    }
    void handleNkeData(const tNkeMsg &msg) override {}
};
*/
class HandleWindSpeed : public N2kNkeHandler
{
public:
    HandleWindSpeed(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
        unsigned char SID;
        // if cant parse we exit
        double windSpeed;
        double windAngle;
        tN2kWindReference windReference;

        /*
        enum tN2kWindReference {
                             N2kWind_True_North=0,     ///< Theoretical Wind (ground referenced, referenced to True North; calculated using COG/SOG)
                             N2kWind_Magnetic=1,       ///< Theoretical Wind (ground referenced, referenced to Magnetic North; calculated using COG/SOG)
                             N2kWind_Apparent=2,       ///< Apparent Wind (relative to the vessel centerline)
                             N2kWind_Apprent=2,        ///< Deprecated - We had the typo in older version of the library
                             N2kWind_True_boat=3,      ///< Theoretical (Calculated to Centerline of the vessel, referenced to ground; calculated using COG/SOG)
                             N2kWind_True_water=4,     ///< Theoretical (Calculated to Centerline of the vessel, referenced to water; calculated using Heading/Speed through Water)
                             N2kWind_Error=6,          ///< error occurred
                             N2kWind_Unavailable=7     ///< unavailable
                           };
        */

        if (ParseN2kWindSpeed(N2kMsg, SID, windSpeed, windAngle, windReference))
        {
            switch (windReference)
            {
            case tN2kWindReference::N2kWind_Error:
            case tN2kWindReference::N2kWind_Unavailable:
                break;
            default:
                // move mstoknots to bridge!!  want data in bridge to be stored as nmea2k less detail in messages

                m_bridge.setWindSpeed(msToKnots(windSpeed));
                m_bridge.setWindAngle(windAngle);
                // samples[currentIndex] = (((msToKnots(windSpeed) / stepsize) * reg[1]) / 100);
                // currentIndex = (currentIndex + 1) % buffersize;
                break;
            }
        }
    }
    void handleNkeData(const tNkeMsg &msg) override
    {
        // convert to right double format and stick it into array
    }
};

class HandlePosition : public N2kNkeHandler
{
public:
    HandlePosition(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
        double Longitude;
        double Latitude;
        if (ParseN2kPGN129025(N2kMsg, Latitude, Longitude))
        {
            m_bridge.setPosition(Latitude, Longitude);
        }
    }
    void handleNkeData(const tNkeMsg &msg) override
    {
        // convert to right double format and stick it into array
    }
};

class HandleCogSog : public N2kNkeHandler
{
public:
    HandleCogSog(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
        unsigned char SID;
        tN2kHeadingReference HeadingReference;
        // tNMEA0183Msg NMEA0183Msg;
        double COG;
        double SOG;
        if (ParseN2kCOGSOGRapid(N2kMsg, SID, HeadingReference, COG, SOG))
        {
            if (HeadingReference == tN2kHeadingReference::N2khr_true)
            {
                // TODO deal with variation!!
                if (!N2kIsNA(SOG))
                {
                    m_bridge.setSpeedOverGround(SOG);
                }
                if (!N2kIsNA(COG))
                {
                    m_bridge.setCourseOverGround(COG);
                }
            }
            // LastCOGSOGTime = millis();
            /*double MCOG = (!N2kIsNA(COG) && !N2kIsNA(Variation) ? COG - Variation : NMEA0183DoubleNA);
            if (HeadingReference == N2khr_magnetic)
            {
                MCOG = COG;
                if (!N2kIsNA(Variation))
                    COG -= Variation;
            }
            if (NMEA0183SetVTG(NMEA0183Msg, COG, MCOG, SOG))
            {
                SendMessage(NMEA0183Msg);
            }*/
        }
        /*
           double Longitude;
           double Latitude;
           if (ParseN2kPGN129025(N2kMsg, Latitude, Longitude))
           {
               m_bridge.setPosition(Latitude, Longitude);
           }*/
    }
    void handleNkeData(const tNkeMsg &msg) override
    {
        // convert to right double format and stick it into array
    }
};

class HandleXte : public N2kNkeHandler
{
public:
    HandleXte(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
        unsigned char SID;
        tN2kXTEMode XTEMode;
        bool NavigationTerminated;
        double XTE;
        if (ParseN2kXTE(N2kMsg, SID, XTEMode, NavigationTerminated, XTE))
        {
            // TODO do what if (NavigationTerminated)
            // XTE is in 0.01m
            // guess the conversion in this case is to meters ?
            // Serial.printf("Xte %dm %f Nm\n", (int)XTE, XTE / 1852);
            if (!N2kIsNA(XTE))
            {
                m_bridge.setXte(XTE);
            }

            /*
            enum tN2kXTEMode  {
                            N2kxtem_Autonomous=0,     ///< autonomous mode
                            N2kxtem_Differential=1,   ///< differential mode
                            N2kxtem_Estimated=2,      ///< estimated mode
                            N2kxtem_Simulator=3,      ///< simulator mode
                            N2kxtem_Manual=4          ///< manual mode
                          };*/
        }
    }
};

// XTE : 129283 Navigation Data : 129284 WP Information : 129285
//  129284
class HandleNavigationData : public N2kNkeHandler
{
public:
    HandleNavigationData(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
        // switch(N2kMsg)
        // an about 50 parameters
        unsigned char SID;
        double DistanceToWaypoint;
        tN2kHeadingReference BearingReference;
        bool PerpendicularCrossed;
        bool ArrivalCircleEntered;
        tN2kDistanceCalculationType CalculationType;
        double ETATime;
        int16_t ETADate;
        double BearingOriginToDestinationWaypoint;
        double BearingPositionToDestinationWaypoint;
        uint32_t OriginWaypointNumber;
        uint32_t DestinationWaypointNumber;
        double DestinationLatitude;
        double DestinationLongitude;
        double WaypointClosingVelocity;
        if (ParseN2kNavigationInfo(N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered, CalculationType,
                                   ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint,
                                   OriginWaypointNumber, DestinationWaypointNumber, DestinationLatitude, DestinationLongitude, WaypointClosingVelocity))
        {
            // Serial.printf("Distance to WP %d\n", (int)DistanceToWaypoint / 1852);
            if (!N2kIsNA(DistanceToWaypoint))
            {
                m_bridge.setDtw(DistanceToWaypoint);
            }
            if (BearingReference == tN2kHeadingReference::N2khr_true)
            {
                // Serial.printf("Course to WP %d\n", (int)((BearingPositionToDestinationWaypoint * 180) / M_PI));
                if (!N2kIsNA(BearingPositionToDestinationWaypoint))
                {
                    m_bridge.setBtw(BearingPositionToDestinationWaypoint);
                }
            }
        }
    };
};

class HandleWp : public N2kNkeHandler
{
public:
    HandleWp(NkeBridge &bridge) : N2kNkeHandler(bridge) {}
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {

        if (N2kMsg.PGN == 129285)
        {
            int index = 0;
            uint16_t start = N2kMsg.Get2ByteUInt(index);
            uint16_t count = N2kMsg.Get2ByteUInt(index);
            uint16_t db_id = N2kMsg.Get2ByteUInt(index);
            uint16_t route_id = N2kMsg.Get2ByteUInt(index);
            uint8_t nav_sup = N2kMsg.GetByte(index);
            uint8_t reserved1 = N2kMsg.GetByte(index);
            if (N2kMsg.GetStr(256, buffer, 256, '\0', index))
            {
                printf("WP Name %s\n", buffer);
            }
            uint8_t reserved2 = N2kMsg.GetByte(index);
            /*
                        for (auto i = 0; i < count; i++)
                        {
                            // WP id, str name, lat long
                        }*/

            // just get the first item

            // index = 10;
        }
    }

private:
    char buffer[256];
};

/*
    //  case 129026UL: HandleCOGSOG(N2kMsg);
    //  case 129029UL: HandleGNSS(N2kMsg);
    //  case 129025UL: HandlePosition(N2kMsg);

    // void tN2kDataToNMEA0183::HandlePosition(const tN2kMsg &N2kMsg) {

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