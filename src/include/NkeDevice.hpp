#pragma once
// #include <cstdint.h>
// #include <stdint.h>
// asuming one channel only has one fast_channel
// might have to make fast id into a var args

// maybe not the right sollution
#include <N2kMessages.h>
#include "NkeChannel.hpp"
#include "NkeBridge.hpp"

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
  void setData(uint16_t val)
  {
    m_data = val;
  }
  void setFastData(uint16_t val) { m_fast_id = val; }

  virtual void handleN2kData(const tN2kMsg & /*N2kMsg*/) {}

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

  // SemaphoreHandle_t  m_sema;
};

namespace Nke
{
  class WaterTemp : public NkeDevice
  {
  public:
    WaterTemp(NkeBridge &bridge)
        : NkeDevice((uint8_t)Nke::Channel::WATER_TEMP, 1, 1), m_bridge(bridge) // todo force in nkedevice ?
    {
      // default offset is 1000 range is 55.5 to 55.5 .. maybe this is the value of the 05 reg ?
      reg[0] = 1000;
    }
    uint16_t data() override
    {
      // return((m_data+reg[0])*reg[1])/100;
      // return m_data+reg[0];
      return uint16_t((m_bridge.waterTemp() + reg[0]) / stepsize);
    }
    /*uint16_t fast_data() override
    {
      return uint16_t((m_bridge.waterTemp() + reg[0]) / stepsize);
    }*/
    /*
        void handleN2kData(const tN2kMsg &N2kMsg) override
        {
          // if cant parse we exit

          // 130312L Temp
          // 130310L OutsideEnviomental

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
            //               tN2kHumiditySource &HumiditySource, double &Humidity, double &AtmosphericPressure
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
          }

          if (tempSource != tN2kTempSource::N2kts_SeaTemperature)
          {
            return;
          }

          if (!N2kIsNA(WaterTemperature))
          {
            setData(uint16_t(KelvinToC(WaterTemperature) / stepsize));
          }
        };
        */
  private:
    // 17.7/1000 (0 => -17.7 and 1000 => 0)
    const float stepsize = 17.7 / 1000;
    NkeBridge &m_bridge;
  };

  // Add one for WindSpeed (windspeed has offset = 100 not 1000 or i did the math wrong)
  // Add one for Boatspeed
  class BoatSpeed : public NkeDevice
  {
  public:
    BoatSpeed(NkeBridge &bridge)
        : NkeDevice((uint8_t)Nke::Channel::BS_AVG, 3, 7, (uint8_t)Nke::Channel::BS_FAST), m_bridge(bridge)
    {
      // default offset is 100 (0-10 with 1000 steps and 1 beeing 100)
      reg[0] = 1000; // this is the standard offset range is -5.38 + 5.38? where 0 is 0 and 5.38 is 2000
      reg[1] = 100;
    }
    // not sure if we should handle config in nke bridge or in end devices..
    // passing config to PGN is probably the correct choice long run
    uint16_t data() override
    {

      auto data = uint16_t(msToKnots(m_bridge.boatSpeed()) / stepsize);

      uint16_t speed_calc = ((data + reg[0]) * reg[1]) / 100;
      //           uint16_t speed_calc = ((m_data + reg[0]) * reg[1]) / 100;
      // Serial.printf("Reading speed %04x\n",speed_calc);
      return speed_calc;
      // return (m_data * reg[1]) / 100;
    }
    uint16_t fastData() override
    {
      auto data = uint16_t(msToKnots(m_bridge.boatSpeed()) / stepsize);

      uint16_t speed_calc = ((data + reg[0]) * reg[1]) / 100;
      // Serial.printf("Reading speed %04x\n",speed_calc);
      return speed_calc;

      // return (m_fast_data * reg[1]) / 100;
    }

    //

  private:
    NkeBridge &m_bridge;
    // 5.38/1000 (0 => -5.38 and 1000 => 0)
    const float stepsize = 5.38 / 1000;
    ;
  };

  // damn..
  // design is more efficient if we can support multiple nke endpoints in one device..
  // rethink NkeDevice ?
  class WindSpeed : public NkeDevice
  {
  public:
    WindSpeed(NkeBridge &bridge)
        : NkeDevice((uint8_t)Nke::Channel::AWS_AVG, 1, 1, (uint8_t)Nke::Channel::AWS_FAST), m_bridge(bridge)
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
      for (auto &s : samples)
      {
        sum += s;
      }
      return uint16_t(sum / buffersize);
    }
    uint16_t fastData() override
    {
      samples[currentIndex] = (((m_bridge.windSpeed() / stepsize) * reg[1]) / 100);
      currentIndex = (currentIndex + 1) % buffersize;
      // bump sample average here instead of centrally or use N2k dampening for value?
      return uint16_t(samples[currentIndex]);
    }

  private:
    NkeBridge &m_bridge;

    static constexpr int buffersize = 10;
    int currentIndex = 0;
    std::array<double, buffersize> samples; // initialize to zero if possible
    // 5.38/1000 (0 => -5.38 and 1000 => 0)
    static constexpr double stepsize = 5.38 / 100;
  };

  class WindAngle : public NkeDevice
  {
  public:
    WindAngle(NkeBridge &bridge) : NkeDevice((uint8_t)Nke::Channel::AWA, 1, 0), m_bridge(bridge) {}
    uint16_t data() override
    {
      // we could store the const ref to wind angle.. and make it faster as it executes in interrupt context
      return m_bridge.windAngle().Nke();
      // return uint16_t(RadToDeg(m_bridge.windAngle()));
      //  const double pi = 3.141592;
      //  double rad = ((heading * pi) / 180.0);
      /*double sum;
      for (auto &s : samples)
      {
        sum += s;
      }
      return uint16_t(sum / buffersize);*/
    }

  private:
    NkeBridge &m_bridge;

    // const double pi = 3.141592;
    //  double rad = ((heading * pi) / 180.0);
  };

  // can i somehow map this better
  class Sog : public NkeDevice
  {
  public:
    Sog(NkeBridge &bridge) : NkeDevice((uint8_t)Nke::Channel::SOG, 1, 0), m_bridge(bridge)
    {
    }

    uint16_t data() override
    {
      return m_bridge.speedOverGround().Nke();
    }

  private:
    NkeBridge &m_bridge;
  };

  class Cog : public NkeDevice
  {
  public:
    Cog(NkeBridge &bridge) : NkeDevice((uint8_t)Nke::Channel::COG, 1, 0), m_bridge(bridge)
    {
    }

    uint16_t data() override
    {
      return m_bridge.courseOverGround().Nke();
    }

  private:
    NkeBridge &m_bridge;
  };

};