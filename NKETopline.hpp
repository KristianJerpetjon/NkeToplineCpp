//TODO make into a singleton!!
#pragma one

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <RingBuf.h>

#include <HardwareSerial.h>
#include <vector>
#include <string>
#include <map>
#include <memory>



#ifndef NKE_TOPLINE_SERIAL 
#define NKE_TOPLINE_SERIAL Serial2
#endif


#if !defined(NKE_TOPLINE_RXD)
#define NKE_TOPLINE_RXD 16
#endif

#if !defined(NKE_TOPLINE_TXD)
#define NKE_TOPLINE_TXD 17
#endif

//#define NKETopline tNKETopline::instance(); 

//Add a dummy byte to make it align better
struct NkeMessage
{
  uint8_t cmd;
  uint8_t data[2];
  uint8_t dummy;
};

class NkeHandler
{
  public:
  NkeHandler(uint8_t id) : m_id(id) {}
  public:
  virtual void handle(const NkeMessage &msg);
  uint8_t id() { return m_id; }
  private:
  uint8_t m_id;
};

class tNKETopline
{
  private:
    tNKETopline(HardwareSerial &serial,const int8_t rxpin=-1,const int8_t txpin=-1);
    
  public:
  enum class State {
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
  //singleton pattern for access..
    static tNKETopline &getInstance() { 
    static tNKETopline instance(NKE_TOPLINE_SERIAL,NKE_TOPLINE_RXD,NKE_TOPLINE_TXD);

    return instance;
    /*static tNKETopline *s_instance=nullptr;

    if (!s_instance) {
      s_instance=(new tNKETopline(NKE_TOPLINE_SERIAL,NKE_TOPLINE_RXD,NKE_TOPLINE_TXD));
    }*/

    //return *s_instance;
  }
  void Open();

  void ParseMessages();
  //probably more efficient mapping array entry to class!
  void AddHandler(std::shared_ptr<NkeHandler> h)  {
    //copy handler into map..
    m_handlers[h->id()]=h;
  }
  private:
  std::map<uint8_t,std::shared_ptr<NkeHandler>> m_handlers;
  //these are all in ISR context!!!
  void receiveByte(uint8_t data);
  void setState(State s);
  void init(uint8_t data);
  void frame(uint8_t data);
  void inter_frame(uint8_t data);
  void print_buf();
  //void find_sync(uint8_t data);
  volatile uint8_t inter_frame_type=0xFF;
  volatile int frame_count=0;

  std::string state2String(State);
  HardwareSerial &m_serial;
  State m_state=State::UNKNOWN;
  int8_t m_rxpin;
  int8_t m_txpin;
  QueueHandle_t m_rxQueue;

  std::vector<uint8_t> m_detected;

  RingBuf<uint8_t, 512> buf;

  bool sync=false;

  uint8_t cmd;
  uint8_t data[5];
  int count=0;

  void debug_frame(const std::string &msg);
};

extern  tNKETopline &NkeTopline;