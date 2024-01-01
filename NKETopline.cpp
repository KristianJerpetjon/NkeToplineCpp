#include "NKETopline.hpp"

#include <map>

  tNKETopline &NkeTopline = tNKETopline::getInstance();


  tNKETopline::tNKETopline(HardwareSerial &serial,int8_t rxpin,int8_t txpin) 
    : m_serial(serial)
    , m_rxpin(rxpin)
    , m_txpin(txpin)
    , m_rxQueue(xQueueCreate(128,sizeof(NkeMessage)))
  {
  }

  void tNKETopline::Open()
  {
    m_serial.begin(9600, SERIAL_8N1, m_rxpin, m_txpin,false,2);    //Hardware Serial of ESP32
    m_serial.setRxTimeout(2);
    //TODO limit what the ISR acesses
    m_serial.onReceive([this](void){
      size_t available = m_serial.available();
      while (available --) {
        receiveByte(m_serial.read());
      }
    });
  }

  void tNKETopline::ParseMessages()
  {
    while(uxQueueMessagesWaiting(m_rxQueue) > 0)
    {
      NkeMessage msg;
      auto ret = xQueueReceive(m_rxQueue,&msg,0);
      //Serial.printf("Received cmd %02x data %02x%02x\n",msg.cmd,msg.data[0],msg.data[1]);

      //Serial.printl()
      //if we fail to read msg return
      if (ret == pdFALSE)
        return;
      auto it = m_handlers.find(msg.cmd);
      if (it != m_handlers.end()) {
        std::shared_ptr<NkeHandler> handler = it->second;
        handler->handle(msg);
      }
      //Dont do anything just empty queue
    }
  }

  const std::map<tNKETopline::State,std::string> stateMap= {
    { tNKETopline::State::UNKNOWN,"UNKNOWN" },
    { tNKETopline::State::INIT,"INIT" },
    { tNKETopline::State::FRAME,"FRAME" },
    { tNKETopline::State::INTER_FRAME,"INTER_FRAME" },
    { tNKETopline::State::FAIL,"FAIL" },
    { tNKETopline::State::SYNC_LOST,"SYNC_LOST" },


  };

  std::string tNKETopline::state2String(State s)
  {
    auto itr=stateMap.find(s);
    if (itr == stateMap.end())
    {
      return "UNKNOWN";
    }
    return itr->second;
  }

  //TODO lock mutex etc 
  void tNKETopline::setState(State state)
  {
    //Serial.printf("Changing state from %s to %s\n",state2String(m_state).c_str(),state2String(state).c_str());
    
    //if we have to reset something when switching state do it here!
    switch(state) {
      case State::INTER_FRAME:
        break;
      case State::FRAME:
        frame_count=0;
        break;
    }
    m_state=state;
  }

  void tNKETopline::init(uint8_t byte) {
    const uint8_t start=0x2;
    const uint8_t end=0xEE;

    static uint8_t expected=start;
    
    static bool detect=false;
    static uint8_t detected=0xFF;
    static int detect_count=0;
    
    if (detect) {
      detect=false;
      if (detected == end)
      {
        setState(State::FRAME);
        Serial.printf("Found %d devices\n",m_detected.size());
        for(auto dev :  m_detected)
        {
            Serial.printf(" %02x",dev);
        }
        Serial.println();

        return;
      }
      m_detected.push_back(detected);
    }
    else if (byte == expected)
    {
      detect=false;
      expected++;
    }
    else {
      detect=true;
      //detect_count=0;
      detected=expected - 1; 
    }

    //TODO if we have an output anounce it here..

  }

   void tNKETopline::frame(uint8_t byte)
   {
    if (count == 0) {
      cmd=byte;
      if (cmd == 0xfc) {
        setState(State::INTER_FRAME);
      }
      //command is from a controller .. in the wrong state
      if (cmd < 0x10) {
        setState(State::INTER_FRAME);
      }
      if ((cmd == 0xbc) | (cmd == 0xbb))
      {
        setState(State::INTER_FRAME);
      }
      count++;
    } else {
      data[count++-1]=byte;
    }

    if (count >= 3) {
      NkeMessage msg;
      msg.cmd=cmd;
      //memcpy(msg.data,data,2);
      for (int i=0;i<2;i++) {
        msg.data[i]=data[i];
      }
      //can we send zero ?
      if (cmd == 0x19)
      {
        debug_frame("Compass from isr");
      }

      //TODO add msg filter here!!
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(m_rxQueue,&msg,&xHigherPriorityTaskWoken);
     /* if (xHigherPriorityTaskWoken)
      {

      }*/
      //if we have a listener..
      frame_count++;
      count=0;
    }

    if (frame_count==10) {
      setState(State::INTER_FRAME);
    }
   }

  void tNKETopline::debug_frame(const std::string &msg)
  {
    Serial.printf("DEBUG %s ",msg.c_str());
    Serial.printf("cmd %02x ",cmd);
    for (int i=1;i<count;i++) {
       Serial.printf(" %02x",data[i-1]);
    }
    Serial.println();
  }

  void tNKETopline::inter_frame(uint8_t byte)
  {
    static uint8_t expected_sync=0xbb;
    static uint8_t function_count=0;

    if (count ==0)
    {
      cmd=byte;
      count++;
    } else 
    { 
      data[(count++)-1]=byte;
    }
    //00 01 //01 02 //etc sequence when controllers > 1
    //else allways 00 00 
    if (cmd < 10) {
      //if we are next controller deal with it .. otherwise ignore..
      if (count==2) {
        //if data==fc.. deal with command
        count=0;
        setState(State::FRAME);
      }
    } 
    else if (cmd == 0xbb | cmd == 0xbc)
    {
      //is this allways the sync pattern ? 
      //bb 00 01 00 
      //bc 00 00 00 
      //frame sync
      if (count == 4) {
        if (cmd != expected_sync) {
          debug_frame("Sync Unexpected");
          //Serial.printf("Unexpected Sync %02x\n",cmd);
        } else {
          //debug_frame("Sync Expected");

        }
        if (cmd == 0xbb){
          expected_sync = 0xbc;
        }else {
          expected_sync = 0xbb;
        }
        count=0;
        setState(State::FRAME);
      }
    }
    //function command fc ? 
    else if (cmd == 0xfc)
    {
      //FC + UPCode + 3 byte payload
      if (count == 5) {
        //setState(State::FAIL);
        debug_frame("Function Command :");
        count=0;

        function_count++;

        if (function_count == 2){
          function_count=0;
          setState(State::FRAME);
        }
        //if second command is seen 
        //setState(State::FRAME);

      }
    }
    else 
    {
      if (count == 4) {
        debug_frame("Unknown inter_frame :");
        setState(State::FAIL);
        count=0;
      }
      //Serial.printf("Unknown inter_frame_type %02x", inter_frame_type);
    }
  }
  void tNKETopline::print_buf()
  {
    int count=0;
    uint8_t e;
    while (buf.pop(e))
    {
      Serial.printf("%02x ",e);
      if (count %16 == 0)
      {
        Serial.println();
      }
      count++;
    }
    Serial.println();
  }
  //receives data in interrupt context!
  void tNKETopline::receiveByte(uint8_t byte)
  {
    //RX buffer for analysis
    uint8_t old;
    if (buf.isFull()) {
      buf.pop(old);
    }

    buf.push(byte);

    static uint8_t prev=0xFF;
    switch(m_state)
    {
      case State::UNKNOWN:
        if (prev == 0x00 && byte == 0xF0)
          setState(State::INIT);
        //if sync pattern.. jump to INTER FRAME
        if ((prev == 0xbc | prev == 0xbb) && byte == 0x00) {  
          count=2;
          cmd=prev;
          data[0]=byte;
          setState(State::INTER_FRAME);
        }
        prev=byte;
        break;
      case State::INIT:
        init(byte);
        break;
      case State::FRAME:
        frame(byte);
        break;
      case State::INTER_FRAME:
        inter_frame(byte);
        break;
      /*case State::FUNCTION:
        function(data);
        break;*/
      case State::FAIL:
        print_buf();
        setState(State::UNKNOWN);
      //case State::SYNC_LOST:
      //  find_sync(data);
        break;
    }
  }