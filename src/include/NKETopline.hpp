// TODO make into a singleton!!
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <RingBuf.h>

#include <SoftwareSerial.h>
#include <vector>
#include <string>
#include <map>
#include <memory>

#ifndef NKE_TOPLINE_SERIAL
#define NKE_TOPLINE_SERIAL Serial2
#endif

#ifndef NKE_TOPLINE_RXD
#define NKE_TOPLINE_RXD GPIO_NUM_16
#endif

#ifndef NKE_TOPLINE_TXD
#define NKE_TOPLINE_TXD GPIO_NUM_17
#endif

#include "NkeDevice.hpp"
#include "NkeHandler.hpp"

class tNKETopline
{
public:
  tNKETopline(HardwareSerial &serial, gpio_num_t rxpin, gpio_num_t txpin);

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
  /*static tNKETopline &getInstance()
  {
    static tNKETopline instance(NKE_TOPLINE_SERIAL);

    return instance;


    // return *s_instance;
  }*/
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

  void printChannels()
  {
    Serial.printf("detected : ");
    for (uint8_t i = 0; i < 0xFF; i++)
    {
      if (m_detected_channels[i] != 0)
      {
        Serial.printf("%02x ", i);
      }
    }
    Serial.printf("\n");

    Serial.printf("main channels : ");
    for (uint8_t i = 0; i < 0xFF; i++)
    {
      if (m_controller_channels[i] != 0)
      {
        Serial.printf("%02x ", i);
      }
    }
    Serial.printf("\n");
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

  unsigned long sendCount() { return m_send_count; }
  unsigned long receiveCount() { return m_receive_count; }
  bool isConnceted() { return m_state == State::FRAME; }

private:
  std::map<uint8_t, std::shared_ptr<NkeHandler>> m_handlers;
  std::vector<std::shared_ptr<NkeDevice>> m_devices;

  std::vector<std::function<void(const Nke::_Message &)>> msgHandlers;

  // removing this as we want to pick up messages that arent only to ourselves
  std::array<NkeDevice *, 256> m_dev_table{};
  // std::array<uint8_t, 256> m_handler_table{};

  // these are all in ISR context!!!
  void receiveByte(uint8_t data);
  void setState(State s);
  void init(uint8_t data);
  void frame(uint8_t data);
  // void inter_frame(uint8_t data);
  void print_buf();
  void sendDevice(uint16_t data);
  // void find_sync(uint8_t data);
  volatile uint8_t inter_frame_type = 0xFF;
  volatile int frame_count = 0;

  std::string state2String(State);
  HardwareSerial &m_serial;
  EspSoftwareSerial::UART m_serialTx;

  State m_state = State::UNKNOWN;
  gpio_num_t m_rxpin;
  gpio_num_t m_txpin;
  QueueHandle_t m_rxQueue;
  QueueHandle_t m_cmdQueue;

  std::vector<uint8_t> m_detected;
  std::vector<uint8_t> m_controller;

  std::array<uint8_t, 256> m_controller_channels;
  std::array<uint8_t, 256> m_detected_channels;
  void clear();

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
  unsigned long m_timeout;                                 // timer to reset to UNKNOWN state if bus is inactive for 1 second
  void updateTimeout(const unsigned long &timeout = 1000); // default 1 second timeout
  bool isTimeout();

  // These are moved from init to be able to reset init if new init sequence F0 is detected!
  const uint8_t start = 0x2;
  const uint8_t end = 0xEE;

  uint8_t expected = start;

  bool detect = false;
  uint8_t detected = 0xFF;
  int detect_count = 0;

  unsigned long m_send_count = 0;
  unsigned long m_receive_count = 0;
};