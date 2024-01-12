// TODO make into a singleton!!
#pragma one

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <RingBuf.h>

#include <SoftwareSerial.h>
// #include <HardwareSerial.h>
#include <vector>
#include <string>
#include <map>
#include <memory>

#include "NkeDevice.hpp"
#include "NkeHandler.hpp"

#ifndef NKE_TOPLINE_SERIAL
#define NKE_TOPLINE_SERIAL Serial2
#endif

#if !defined(NKE_TOPLINE_RXD)
#define NKE_TOPLINE_RXD 16
#endif

#if !defined(NKE_TOPLINE_TXD)
#define NKE_TOPLINE_TXD 17
#endif

struct NkeMessage
{
  uint8_t cmd;
  uint8_t data[2];
  uint8_t dummy;
};

static inline void uint16ToChars(uint16_t in, uint8_t *out)
{
  // depends on the endians
  out[0] = (in >> 8) & 0xFF;
  out[1] = in & 0xFF;
}

static inline void charsTouint16(uint8_t *in, uint16_t &out)
{
  out = (in[0] << 8) | (in[1]);
}

class tNKETopline
{
private:
  tNKETopline(HardwareSerial &serial, const int8_t rxpin = -1, const int8_t txpin = -1);

public:
  enum class State
  {
    UNKNOWN,
    INIT,
    FRAME,
    INTER_FRAME,
    FAIL,
    SYNC_LOST,
    FUNCTION,
  };

  tNKETopline(const tNKETopline &) = delete; // no copying
  tNKETopline &operator=(const tNKETopline &) = delete;
  static tNKETopline &getInstance()
  {
    static tNKETopline instance(NKE_TOPLINE_SERIAL, NKE_TOPLINE_RXD, NKE_TOPLINE_TXD);

    return instance;
    /*static tNKETopline *s_instance=nullptr;

    if (!s_instance) {
      s_instance=(new tNKETopline(NKE_TOPLINE_SERIAL,NKE_TOPLINE_RXD,NKE_TOPLINE_TXD));
    }*/

    // return *s_instance;
  }
  void Open();

  void ParseMessages();
  // probably more efficient mapping array entry to class!
  void addHandler(std::shared_ptr<NkeHandler> handler)
  {
    // copy handler into map..
    m_handlers[handler->id()] = handler;
  }

  // TODO make thread safe
  void addDevice(std::shared_ptr<NkeDevice> dev)
  {
    m_devices.push_back(dev);
    m_dev_table[dev->id()] = dev.get();
    Serial.printf("Adding %02x %p\n",dev->id(),dev.get());
    if (dev->fast_id() != 0)
    {
      m_dev_table[dev->fast_id()] = dev.get();
    }
  };

private:
  std::map<uint8_t, std::shared_ptr<NkeHandler>> m_handlers;
  std::vector<std::shared_ptr<NkeDevice>> m_devices;
  //TODO store as reference wrappers instead of pointers ? 
  std::array<NkeDevice *, 256> m_dev_table{};

  // these are all in ISR context!!!
  void receiveByte(uint8_t data);
  void setState(State s);
  void init(uint8_t data);
  void frame(uint8_t data);
  void inter_frame(uint8_t data);
  void print_buf();
  void sendDevice(uint16_t data);
  // void find_sync(uint8_t data);
  volatile uint8_t inter_frame_type = 0xFF;
  volatile int frame_count = 0;

  std::string state2String(State);
  HardwareSerial &m_serial;
  EspSoftwareSerial::UART m_serialTx;

  State m_state = State::UNKNOWN;
  int8_t m_rxpin;
  int8_t m_txpin;
  QueueHandle_t m_rxQueue;

  std::vector<uint8_t> m_detected;

  RingBuf<uint8_t, 512> buf;

  bool sync = false;

  // maybe we can make this smarter.. ?
  uint8_t cmd;
  uint8_t data[5];
  int count = 0;

  void debug_frame(const std::string &msg);
};

extern tNKETopline &NkeTopline;