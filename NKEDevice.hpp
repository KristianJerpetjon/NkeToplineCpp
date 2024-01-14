#pragma once
// #include <cstdint.h>
// #include <stdint.h>
// asuming one channel only has one fast_channel
// might have to make fast id into a var args

// maybe not the right sollution
#include <N2kMessages.h>

static inline uint16_t charToUint16(uint8_t *data)
{
  return data[0] << 8 | data[1] & 0xFF;
}

static inline void uint16ToChar(const uint16_t &val, uint8_t *data)
{
  data[0] = (val >> 8) & 0xFF;
  data[1] = val & 0xFF;
}

// is fast device part of device or separate subclass ?
class NkeDevice
{
public:
  static constexpr auto max_regs = 16;

  NkeDevice(uint8_t id, uint8_t major, uint8_t minor, uint8_t fast_id = 0)
      : m_id(id), m_fast_id(fast_id), m_version(256 + major * 10 + minor) //, m_sema(xSemaphoreCreateBinary())
  {
    for (auto &a : reg)
    {
      a = 0;
    }
    m_data = 0;
    m_fast_data = 0;
  }
  void setData(uint16_t val) { 
    m_data = val; 
  }
  void setFastData(uint16_t val) { m_fast_id = val; }

  virtual void handleN2kData(const tN2kMsg &/*N2kMsg*/) {}

  bool isFast(uint8_t id) { return id == m_fast_id; }
  virtual uint16_t data() { return m_data; }
  virtual uint16_t fastData() { return m_fast_data; }
  uint16_t version() { return m_version; }

  // for fast access ??
  std::array<uint16_t, max_regs> reg = {};
  // uint16_t reg[max_regs];
  // maybe we need more
  uint8_t id() { return m_id; }
  uint8_t fast_id() { return m_fast_id; }

private:
  uint8_t m_id;
  uint8_t m_fast_id;
  // can be in theory 256 but lets see.. 0x11 is the reg in autopilot.. so probably need to bump at some point
  uint16_t m_version;

protected:
  volatile uint16_t m_fast_data;
  volatile uint16_t m_data;

  //SemaphoreHandle_t  m_sema; 

};

namespace Nke
{
  class Temp : public NkeDevice
  {
  public:
    Temp(uint16_t id, uint8_t version_high, uint8_t version_low) : NkeDevice(id, version_high, version_low)
    {
      // default offset is 1000 range is 55.5 to 55.5 .. maybe this is the value of the 05 reg ?
      reg[0] = 1000;
    }
    uint16_t data() override
    {
      //return((m_data+reg[0])*reg[1])/100;
      return m_data+reg[0];
    }
    
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
      //if cant parse we exit

      //130312L Temp
      //130310L OutsideEnviomental
      unsigned char SID;
      double WaterTemperature;
      double OutsideAmbientAirTemperature;
      double AtmosphericPressure;
    //Parse 

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
    //double ActualTemperature;
    double SetTemperature;

    switch(N2kMsg.PGN)
    {
      case 130310L:
      if (!ParseN2kOutsideEnvironmentalParameters(N2kMsg,SID,WaterTemperature,OutsideAmbientAirTemperature,AtmosphericPressure) ) {
        return;
      }
      tempSource = tN2kTempSource::N2kts_SeaTemperature;
      break;
      case 130311L:

      /*const tN2kMsg &N2kMsg, unsigned char &SID, tN2kTempSource &TempSource, double &Temperature,
                     tN2kHumiditySource &HumiditySource, double &Humidity, double &AtmosphericPressure*/
      if (!ParseN2kEnvironmentalParameters(N2kMsg,SID,tempSource,WaterTemperature,humiditySource,humidity,AtmosphericPressure))
      {
        return;
      }
      break;
      case 130316L:
      if (!ParseN2kTemperatureExt(N2kMsg,SID,instance,tempSource,WaterTemperature,SetTemperature))
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

    if (tempSource != tN2kTempSource::N2kts_SeaTemperature) {
      return;
    }


    if (!N2kIsNA(WaterTemperature))
    { 
      setData(uint16_t(KelvinToC(WaterTemperature) / stepsize));
    }
    };
    /*
      if (ParseN2kBoatSpeed(N2kMsg, SID, SOW, SOG, SWRT))
      {
        if (N2kIsNA(SOW))
        {
          setData(0);
        }
        else {
          //We do the math in knots .. 
          setData(uint16_t(msToKnots(SOW) / stepsize));
          m_fast_data=m_data;
        }
      }
    };*/
    private:
      //17.7/1000 (0 => -17.7 and 1000 => 0)
      const float stepsize = 17.7/1000;


  };

  //Add one for WindSpeed (windspeed has offset = 100 not 1000 or i did the math wrong)
  //Add one for Boatspeed
  class BoatSpeed : public NkeDevice
  {
  public:
    BoatSpeed(uint16_t id, uint8_t version_high, uint8_t version_low, uint8_t fast_speed = 0)
        : NkeDevice(id, version_high, version_low, fast_speed)
    {
      // default offset is 100 (0-10 with 1000 steps and 1 beeing 100)
      reg[0] = 1000; // this is the standard offset range is -5.38 + 5.38? where 0 is 0 and 5.38 is 2000
      reg[1] = 100;
    }
    // not sure if we should handle config in nke bridge or in end devices..
    // passing config to PGN is probably the correct choice long run
    uint16_t data() override
    {
      uint16_t speed_calc=((m_data+reg[0])*reg[1])/100;
      //Serial.printf("Reading speed %04x\n",speed_calc);
      return speed_calc;
      //return (m_data * reg[1]) / 100;
    }
    uint16_t fastData() override
    {
      uint16_t speed_calc=((m_data+reg[0])*reg[1])/100;
      //Serial.printf("Reading speed %04x\n",speed_calc);
      return speed_calc;
      
      //return (m_fast_data * reg[1]) / 100;
    }

    //
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
      unsigned char SID;
      double SOW;
      double SOG;
      tN2kSpeedWaterReferenceType SWRT;
      //if cant parse we exit
      if (ParseN2kBoatSpeed(N2kMsg, SID, SOW, SOG, SWRT))
      {
        if (N2kIsNA(SOW))
        {
          setData(0);
        }
        else {
          //We do the math in knots .. 
          setData(uint16_t(msToKnots(SOW) / stepsize));
          m_fast_data=m_data;
        }
      }
    };
  private:
    //5.38/1000 (0 => -5.38 and 1000 => 0)
    const float stepsize = 5.38/1000;;
  };

  //damn.. 
  //design is more efficient if we can support multiple nke endpoints in one device..
  //rethink NkeDevice ?
  class WindSpeed : public NkeDevice
  {
  public:
    WindSpeed(uint16_t id, uint8_t version_high, uint8_t version_low, uint8_t fast_speed = 0)
        : NkeDevice(id, version_high, version_low, fast_speed)
    {
      // default offset is 100 (0-10 with 1000 steps and 1 beeing 100)
      reg[0] = 0; // this is the standard offset range is -5.38 + 5.38? where 0 is 0 and 5.38 is 2000
      reg[1] = 100;
    }
    // not sure if we should handle config in nke bridge or in end devices..
    // passing config to PGN is probably the correct choice long run
    uint16_t data() override
    {
      double sum;
      for(auto &s : samples)
      {
        sum+=s;
      }
      return uint16_t(sum/buffersize);
    }
    uint16_t fastData() override
    {
      return uint16_t(samples[currentIndex]);
    }
   /*
    void setFastData(uint16_t data)
    {
      samples[currentIndex]=data;
      currentIndex=(currentIndex+1)% buffersize;
    }
*/
    //
    void handleN2kData(const tN2kMsg &N2kMsg) override
    {
      unsigned char SID;
      //if cant parse we exit
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

      if (ParseN2kWindSpeed(N2kMsg, SID, windSpeed,windAngle, windReference))
      {
        switch(windReference)
        {
          case tN2kWindReference::N2kWind_Error:
          case tN2kWindReference::N2kWind_Unavailable:
          break;
          default:
                samples[currentIndex]=(((msToKnots(windSpeed)/stepsize)*reg[1])/100);
                currentIndex=(currentIndex+1)% buffersize;
          break;
        }
      }
    };
  private:
      static constexpr int buffersize=10;
      int currentIndex=0;
      std::array<double,buffersize> samples; //initialize to zero if possible
    //5.38/1000 (0 => -5.38 and 1000 => 0)
    static constexpr double stepsize = 5.38/100;
  };
};