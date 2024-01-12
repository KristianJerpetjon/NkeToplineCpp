#pragma once
// #include <cstdint.h>
// #include <stdint.h>
// asuming one channel only has one fast_channel
// might have to make fast id into a var args

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
      : m_id(id), m_fast_id(fast_id), m_version(256 + major * 10 + minor)
  {
    for (auto &a : reg)
    {
      a = 0;
    }
    m_data = 0;
    m_fast_data = 0;
  }
  void setData(uint16_t val) { m_data = val; }
  void setFastData(uint16_t val) { m_fast_id = val; }

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
  uint16_t m_fast_data;
  uint16_t m_data;
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
    }

    /*    void setTemp(uint16_t temp)
        {
          m_data = temp;
          // if we assume temp is given in value of 0.1 degrees
          // f(x)=
          // m_data = ((temp * 1000) - reg[0])
        }*/
  };
  class Speed : public NkeDevice
  {
  public:
    Speed(uint16_t id, uint8_t version_high, uint8_t version_low, uint8_t fast_speed = 0)
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

      return (m_data * reg[1]) / 100;
    }
    uint16_t fastData() override
    {
      const float stepsize = 0.00538;

      return (m_fast_data * reg[1]) / 100;
    }

    void setSpeed(double speed)
    {
      m_data = uint16_t(speed / stepsize);
    }

  private:
    const float stepsize = 0.00538;

    /*
    void setSpeed(uint16_t speed)
    {
      m_data = (speed * reg[1]) / 100;
    }
    void setFastSpeed(uint16_t speed)
    {
      m_fast_data = (speed * reg[1]) / 100;
    }*/
  };
};