// TODO make into a singleton!!
#pragma once

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

/*
struct NkeMessage
{
  uint8_t cmd;
  uint8_t data[2];
  uint8_t dummy;
};*/

/*
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
*/
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
    // m_handler_table[handler->id()]=0xFF; //ff means there is a handler for this id.. //this provides low level filtering but doesnt make sense for controllers..
    // also at a rate < 3ms per message its not a biggie
  }

  // TODO make thread safe
  void addDevice(std::shared_ptr<NkeDevice> dev)
  {
    m_devices.push_back(dev);
    m_dev_table[dev->id()] = dev.get();
    Serial.printf("Adding %02x %p\n", dev->id(), dev.get());
    if (dev->fast_id() != 0)
    {
      m_dev_table[dev->fast_id()] = dev.get();
    }
  };

  // this adds a raw handler that receives all messages
  void addMsgHandler(std::function<void(const Nke::_Message &)> handler)
  {
    msgHandlers.push_back(handler);
  }

  bool sendCommand(const Nke::_Message &msg)
  {
    // try sending and dont wait
    return xQueueSend(m_cmdQueue,
                      (void *)&msg,
                      (TickType_t)0) == pdPASS;
  }

private:
  std::map<uint8_t, std::shared_ptr<NkeHandler>> m_handlers;
  std::vector<std::shared_ptr<NkeDevice>> m_devices;

  std::vector<std::function<void(const Nke::_Message &)>> msgHandlers;
  // std::queue<Nke::_Message>
  // is there any point in being more than one controller in the network ? ..
  // my guess is not!
  // todo change these to use xqueues
  // std::deque<Nke::_Message> m_command_queue;
  // std::deque<Nke::_Message> reply; //TODO ship replies over same queue as data ?  use nkehandler or similar

  // TODO store as reference wrappers instead of pointers ?
  std::array<NkeDevice *, 256> m_dev_table{};
  // std::array<uint8_t, 256> m_handler_table{};

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
  QueueHandle_t m_cmdQueue;

  std::vector<uint8_t> m_detected;

  RingBuf<uint8_t, 512> buf;

  bool sync = false;

  // maybe we can make this smarter.. ?
  uint8_t cmd;
  uint8_t data[5];
  int count = 0;

  void debug_frame(const std::string &msg);
  uint8_t m_active_controller = 0x00;
  uint8_t function_count = 0;

  void channel_decoder(uint8_t channel, int mark = 0);
  void handle_functions();
  void handle_bx();
  void handle_channel();
  void handle_controllers();
  void sendFx(const Nke::_Message &msg);
  uint8_t channel;
};

extern tNKETopline &NkeTopline;