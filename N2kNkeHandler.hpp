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
            // if (N2kIsNA(SOW))
            // {
            m_bridge.setBoatSpeed(SOW);
            // setData(0);
            /* }
             else
             {

                 // We do the math in knots ..
                 setData(uint16_t(msToKnots(SOW) / stepsize));
                 m_fast_data = m_data;
             }*/
            m_bridge.setSpeedOverGround(SOG);
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
              //move mstoknots to bridge!!  want data in bridge to be stored as nmea2k less detail in messages

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