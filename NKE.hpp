#pragma once

#include <vector>
#include <map>
#include <memory>
#include <deque>
#include <list>
#include <algorithm>
#include <functional>

const uint8_t SLAVE_ID = 4;
const bool SLAVE_ENABLED=false;


//TODO use nke namespace
class nke_data
{

  public:
  nke_data(uint8_t request) : m_request(request) {}
  virtual void handle_data(uint8_t request,uint8_t *response);
  uint8_t request() { return m_request; }
  private:
    uint8_t m_request;
};

class depth_handler : public nke_data
{
  depth_handler() : nke_data(0x16){}
  void handle_data(uint8_t request,uint8_t *response) override {

  }
};

using nke_handler=std::function<void(uint8_t,uint8_t*)>;
class nke
{
  enum class Decoder
  {
    NOSYNC=0,
    DATA=1,
    SYNC=2,
    COMMAND=3,
  };
  public:
    void protocol_decoder(uint8_t byte);
    bool handle_data(std::deque<uint8_t> &data);
    bool handle_controller(std::deque<uint8_t> &data);
    bool handle_command(std::deque<uint8_t> &data);
    bool sync_sequence(uint8_t data);
    void update_expected(uint8_t current);
    bool is_us();
    void add_decoder(uint8_t request,nke_handler handler)
    {
      m_handlers.insert({request,handler});
    }
  private:
    uint8_t request;
    uint8_t response[2];
    std::vector<uint8_t> m_sensors={};
    std::map<uint8_t,nke_handler> m_handlers={};
    Decoder m_decoder=Decoder::NOSYNC;
    std::vector<uint8_t> m_sync_pattern={0xb8,0xb9,0xba,0xbb,0xbc,0xcb};
    std::list<uint8_t> m_controllers;
    std::deque<uint8_t> m_last_data;
    //std::iterator<std::list<uint8_t>> m_next;
    bool m_all_controllers=false;
    uint8_t m_expected=0;
    uint8_t m_us=SLAVE_ID;
    std::deque<uint8_t> m_rxbuff;

};