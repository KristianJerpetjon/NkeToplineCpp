#include "NKETopline.hpp"
#include "NkeMessage.hpp"

#include <map>

/*tNKETopline &NkeTopline = tNKETopline::getInstance();
*/
tNKETopline::tNKETopline(HardwareSerial &serial, gpio_num_t  rxpin, gpio_num_t  txpin)
    : m_serial(serial), m_rxpin(rxpin), m_txpin(txpin), m_rxQueue(xQueueCreate(128, sizeof(Nke::_Message))), m_cmdQueue(xQueueCreate(128, sizeof(Nke::_Message)))
{
}

//TODO evaluate if running only SWSerial makes more sense
void tNKETopline::Open()
{

  //TODO abstract print to callback with id's
  Serial.printf("Starting NkeTopline 9600 baud RX_PIN %d,TX_PIN %d\n",m_rxpin,m_txpin);

  m_serial.setRxTimeout(1);
  // TODO limit what the ISR acesses
  m_serial.onReceive([this](void)
                     {
      size_t available = m_serial.available();
      while (available --) {
        //auto byte=m_serial.read();
        //receiveByte(byte);
        //Serial.printf("%02x\n",byte);
        receiveByte(m_serial.read());
      } });


  m_serial.begin(9600, SERIAL_8E1, m_rxpin); // Hardware Serial of ESP32
  
  // TODO check if it matters if its 8S1 vs 8N1 as long as we use write(byte,parity)
  m_serialTx.begin(9600, SWSERIAL_8S1, -1, m_txpin);

  //set timeout to 1 second
}

void tNKETopline::ParseMessages()
{

  //Todo if no data .. Reset NKE status

  while (uxQueueMessagesWaiting(m_rxQueue) > 0)
  {
    m_receive_count++;
    Nke::_Message msg;
    auto ret = xQueueReceive(m_rxQueue, &msg, 0);

    // find handler in handlers ? or just call callback.. think the latter is even better
    //  Serial.printf("Received cmd %02x data %02x%02x\n",msg.cmd,msg.data[0],msg.data[1]);

    for (auto &handler : msgHandlers)
    {
      handler(msg);
    }

    // Serial.printl()
    // if we fail to read msg return
    if (ret == pdFALSE)
      return;
    auto it = m_handlers.find(msg.channel);
    if (it != m_handlers.end())
    {
      std::shared_ptr<NkeHandler> handler = it->second;
      // uint16_t data = charToUint16(msg.data);
      handler->handle(msg);
    }
    // Dont do anything just empty queue
  }
  //probably should reset more
  if (isTimeout())
  {
    setState(State::UNKNOWN);
  }
}

const std::map<tNKETopline::State, std::string> stateMap = {
    {tNKETopline::State::UNKNOWN, "UNKNOWN"},
    {tNKETopline::State::INIT, "INIT"},
    {tNKETopline::State::FRAME, "FRAME"},
    {tNKETopline::State::INTER_FRAME, "INTER_FRAME"},
    {tNKETopline::State::FAIL, "FAIL"},
    {tNKETopline::State::SYNC_LOST, "SYNC_LOST"},

};

void tNKETopline::updateTimeout(const unsigned long &timeout)
{
      m_timeout=millis()+timeout;
}

bool tNKETopline::isTimeout() 
{
  if (m_state != State::UNKNOWN)
  {
    if (m_timeout< millis() )
    {
      Serial.printf("Nke bus Timeout\n");
      return true;
    }
  }
  return false;
}

std::string tNKETopline::state2String(State s)
{
  auto itr = stateMap.find(s);
  if (itr == stateMap.end())
  {
    return "UNKNOWN";
  }
  return itr->second;
}

// TODO lock mutex etc
void tNKETopline::setState(State state)
{
  // Serial.printf("Changing state from %s to %s\n",state2String(m_state).c_str(),state2String(state).c_str());

  // if we have to reset something when switching state do it here!
  switch (state)
  {
    case State::INIT:
    updateTimeout(10000); //wait up to 10 seconds for init sequence
    break;
  case State::INTER_FRAME:
    break;
  case State::FRAME:
    //start timer!
    updateTimeout();
    frame_count = 0;
    break;
  case State::FAIL:
    frame_count = 0;
    count = 0;
    // todo turn sync off!
    break;
  }
  m_state = state;
}

void tNKETopline::sendDevice(uint16_t data)
{
  m_send_count++;
  m_serialTx.write((data >> 8) & 0xFF, PARITY_SPACE);
  m_serialTx.write(data & 0xFF, PARITY_SPACE);
}

// todo make init care about "_ or high parity !!"
void tNKETopline::init(uint8_t byte)
{
  if (detect)
  {
    detect = false;
    m_detected.push_back(detected);
  }
  else if (byte == expected)
  {
    // dont know if ee can exist but if not then after EE its intra frame to begin with either 00 00 or 00 + first slave
    if (expected == end)
    {
      count = 0;
      setState(State::FRAME);
      Serial.printf("Found %d devices\n", m_detected.size());
      for (auto dev : m_detected)
      {
        Serial.printf(" %02x", dev);
      }
      Serial.println();
      return;
    }
    detect = false;
    expected++;
    auto dev = m_dev_table[byte];
    if (dev != nullptr)
    {
      // TODO FIXME add debug flag for this
      Serial.printf("Announcing %02x\n", byte);
      if (dev->isFast(byte))
        sendDevice(dev->fastData());
      else
        sendDevice(dev->data());
    }
  }
  else
  {
    detect = true;
    // detect_count=0;
    detected = expected - 1;
  }
}

void tNKETopline::frame(uint8_t byte)
{
  if (count == 0)
  {
    cmd = byte;
    auto dev = m_dev_table[cmd];
    // TODO send out side of interrupt handler
    if (dev != nullptr)
    {
      if (dev->isFast(byte))
        sendDevice(dev->fastData());
      else
        sendDevice(dev->data());
    }

    if (cmd == 0xfc | cmd == 0xF1 | cmd == 0xF4)
    {
      // a controller thats not master has sendt a command at the end of the intra frame
      if (frame_count == 0)
      {
        function_count++;
        setState(State::INTER_FRAME);
      }
    }
    // command is from a controller .. in the wrong state
    if (cmd < 0x10)
    {
      setState(State::INTER_FRAME);
    }
    if ((cmd == 0xbc) | (cmd == 0xbb))
    {
      setState(State::INTER_FRAME);
    }
    count++;
  }
  else
  {
    data[count++ - 1] = byte;
  }

  if (count >= 3)
  {
    // NkeMessage msg;
    // ok lets put more stuff into this as wel go
    Nke::_Message msg;
    msg.channel = cmd;
    msg.len = 2;
    memcpy(msg.data, data, 2);
    /*
        msg.cmd = cmd;
        memcpy(msg.data, data, 2);
    */
    // TODO add msg filter here!!
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(m_rxQueue, &msg, &xHigherPriorityTaskWoken);
    /* if (xHigherPriorityTaskWoken)
     {

     }*/
    // if we have a listener..
    frame_count++;
    count = 0;
  }

  if (frame_count == 10)
  {
    setState(State::INTER_FRAME);
    // reset on frame completion what the active controller is.. asumption is only master can send at start of intra frame
    // Not done in the setState because of the fx command following controller at the end resulting in an error.
    // we could also do a detect not fx or 0x to return to frame state.. might be a better design
    m_active_controller = 0x00;
  }
}

void tNKETopline::debug_frame(const std::string &msg)
{
  Serial.printf("DEBUG %s ", msg.c_str());
  Serial.printf("cmd %02x ", channel);
  for (int i = 1; i < count; i++)
  {
    Serial.printf(" %02x", data[i - 1]);
  }
  Serial.println();
}

void tNKETopline::handle_channel()
{
  // one byte received
  if (count == 1)
  {
    auto dev = m_dev_table[channel];
    // TODO send out side of interrupt handler
    if (dev != nullptr)
    {
      if (dev->isFast(channel))
        sendDevice(dev->fastData());
      else
        sendDevice(dev->data());
    }
  }

  if (count >= 3)
  {
      //send all messages up the stack
      Nke::_Message msg;
      msg.channel = channel;
      msg.len = 2;
      //probably faster to do direct assignment
      memcpy(msg.data, data, 2);
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(m_rxQueue, &msg, &xHigherPriorityTaskWoken);
    count = 0;
  }
}

void tNKETopline::handle_bx()
{
  //What are in these messages ? .. Timer ? position ? 
  switch (channel)
  {
  case 0xb9:
  case 0xba:
  case 0xbb:
  case 0xbc:
    if (count == 3)
    {
      count = 0;
    }
    break;
  default:
    Serial.printf("WARNING unknown bx %02x, %02x, %02x\n", channel, data[0], data[1]);
    break;
  }
}

void tNKETopline::handle_functions()
{
  //send all fc messages up in the system
  if (count == 5)
  {
      Nke::_Message msg;
      msg.channel = channel;
      msg.len = 4;
      memcpy(msg.data, data, 4);
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xQueueSendFromISR(m_rxQueue, &msg, &xHigherPriorityTaskWoken);
  }
  //we can move the reg setting async but the send needs to stay for both fc and f1
  switch (channel)
  {
  case 0xf4:
    // F4 means WRITE!!!
    if (count == 5)
    {
      auto id = data[0];
      auto reg = data[1];
      auto dev = m_dev_table[id];
      if (dev != nullptr)
      {
        if (reg < NkeDevice::max_regs)
        {
          dev->reg[reg] = charToUint16(&data[2]);
        }
      }
      debug_frame("write:");
      count = 0;
    }
    break;
    // FC means read!!

  case 0xfc:
    if (count == 3)
    {
      auto id = data[0];
      auto reg = data[1];
      auto dev = m_dev_table[id];
      if (dev != nullptr)
      {
        if (reg < NkeDevice::max_regs)
        {
          sendDevice(dev->reg[reg]);
        }
      }
    }
    if (count == 5)
    {
      debug_frame("Read:");
      count = 0;
    }
    break;
  // protocol does write read verify ... from the looks of things.

  // fc requires emulators to "REPLY"
  case 0xf1:

    if (count == 2)
    {
      auto id = data[0];
      auto dev = m_dev_table[id];
      if (dev != nullptr)
      {
        sendDevice(dev->version());
        // why the 0x40 ?
        m_serialTx.write(0x40, PARITY_SPACE);
      }
    }
    if (count == 5)
    {
      debug_frame("F1 Command :");
      count = 0;
    }
    break;
  case 0xFF:
    Serial.printf("Lost sync\n");
    print_buf();
    setState(State::UNKNOWN);
    break;

  default:
    if (count == 5)
    {
      debug_frame("Unhandled Intra :");
      // function_count++;
      count = 0;
      // we are not working..
      // setState(State::FAIL);
    }
  }
}

void tNKETopline::sendFx(const Nke::_Message &msg)
{
  m_send_count++;
  if (msg.len > 0)
  {
    // i have no idea if this is correct
    m_serialTx.write(msg.channel, PARITY_MARK);
    for (auto i = 0; i < msg.len; i++)
    {
      // send data not as "controller"
      m_serialTx.write(msg.data[i], PARITY_SPACE);
    }
    // for(auto &c : msg.data)
  }
}

void tNKETopline::handle_controllers()
{
  // auto id = data[0];
  if (count == 1)
  {
    //if our controller ID is the state we are in send message..
    //currently we only sopport one controller / slave id.. should be enough
    auto dev = m_dev_table[channel];
    if (dev != nullptr)
    {
      BaseType_t xTaskWokenByReceive = pdFALSE;
      Nke::_Message msg;
      if (uxQueueMessagesWaitingFromISR(m_cmdQueue) > 0)
      {
        if (xQueueReceiveFromISR(m_cmdQueue,
                                 (void *)&msg,
                                 &xTaskWokenByReceive))
        {
          sendFx(msg);
        }
      }
    }
    // if
  }
  count = 0;
}

// TODO check mark and space of incoming data..!!
void tNKETopline::channel_decoder(uint8_t byte, int mark)
{
  if (count == 0)
  {
    // TODO check that MARK is correct for controller
    channel = byte;
  }
  else
  {
    // maybe it should all be in data ?
    data[count - 1] = byte;
  }
  count++;

  if (channel >= 0xF0)
  {
    handle_functions();
  }
  else if (channel <= 0xBF && channel > 0xb0)
  {
    handle_bx();
  }
  else if (channel < 0x10)
  {

    handle_controllers();
    // TODO if we have a controller send data..
    // if command in command queue send it if we are controller cmd
  }
  else
  {
    // maybe split in handle fast channel and handle slow at some point
    handle_channel();
  }
}

void tNKETopline::inter_frame(uint8_t byte)
{
  if (count == 0)
  {
    cmd = byte;
    count++;
  }
  else
  {
    data[(count++) - 1] = byte;
  }

  switch (cmd)
  {
  // bb issued by controller with 2 byte payload but what is it.. some contain other things so maybe we should study this more..
  // can bb only be the first of the two control frame
  case 0xb9:
  case 0xba:
  case 0xbb:
  case 0xbc:
    if (count == 3)
    {
      // debug_frame("bx :");
      //Serial.printf("bx %02x, %02x, %02x\n", cmd, data[0], data[1]);
      //We need to figure out what BX os
      function_count++;
      count = 0;
    }
    break;
  case 0xf4:
    // F4 means WRITE!!!
    if (count == 5)
    {
      auto id = data[0];
      auto reg = data[1];
      auto dev = m_dev_table[id];
      if (dev != nullptr)
      {
        if (reg < NkeDevice::max_regs)
        {
          dev->reg[reg] = charToUint16(&data[2]);
        }
      }
      debug_frame("write:");
      if (m_active_controller == 0x00) // if not master dont count the frame also verify that master only can occupy the timeslot
        function_count++;
      count = 0;
    }
    break;
  case 0xfc:
    if (count == 3)
    {
      auto id = data[0];
      auto reg = data[1];
      auto dev = m_dev_table[id];
      if (dev != nullptr)
      {
        if (reg < NkeDevice::max_regs)
        {
          sendDevice(dev->reg[reg]);
        }
      }
    }
    if (count == 5)
    {
      debug_frame("Read:");
      if (m_active_controller == 0x00) // if not master dont count the frame also verify that master only can occupy the timeslot
        function_count++;
      count = 0;
    }
    break;
  // FC means read!!
  // protocol does write read verify ... from the looks of things.
  // fc requires emulators to "REPLY"
  case 0xf1:

    if (count == 2)
    {
      auto id = data[0];
      auto dev = m_dev_table[id];
      if (dev != nullptr)
      {
        sendDevice(dev->version());
        // why the 0x40 ?
        m_serialTx.write(0x40, PARITY_SPACE);
      }
    }
    if (count == 5)
    {
      debug_frame("F1 Command :");
      if (m_active_controller == 0x00) // if not master dont count the frame also verify that master only can occupy the timeslot
      {
        function_count++;
      }
      count = 0;
    }
    break;
  default:
    if (cmd < 0x10)
    {
      if (cmd == 0)
      {
        Serial.printf("Master 00\n");
      }
      else
      {
        Serial.printf("Slave %d\n", cmd);
      }
      m_active_controller = cmd;
      // Serial.printf("Handled control from %d\n",)
      function_count++;
      count = 0;
    }
    else
    {
      if (count == 5)
      {
        debug_frame("Unhandled Intra :");
        function_count++;
        count = 0;
        // we are not working..
        setState(State::FAIL);
        return;
      }
    }
    break;
  }

  if (function_count == 2)
  {
    function_count = 0;
    setState(State::FRAME);
    return;
  }

}
void tNKETopline::print_buf()
{
  int count = 0;
  uint8_t e;
  while (buf.pop(e))
  {
    Serial.printf("%02x ", e);
    count++;
    if (count % 16 == 0)
    {
      Serial.println();
    }
  }
  Serial.println();
}
// receives data in interrupt context!
void tNKETopline::receiveByte(uint8_t byte)
{
  // RX buffer for analysis
  uint8_t old;
  if (buf.isFull())
  {
    // print_buf();
    buf.pop(old);
  }

  buf.push(byte);

  static uint8_t prev = 0xFF;
  switch (m_state)
  {
  case State::UNKNOWN:
    if (prev == 0xF0 && byte == 0x02)
    {
      Serial.printf("INIT Detected\n");
      expected = start;
      detect = false;
      detected = 0xFF;
      detect_count = 0;
      setState(State::INIT);
      init(byte);
      break;
    }
    // if sync pattern.. jump to Framing
    if ((prev == 0xbc | prev == 0xbb | prev == 0xba | prev == 0xb9) && byte == 0x00)
    {
      channel = prev;
      data[0] = byte;
      setState(State::FRAME);
      break;
    }
    prev = byte;
    break;
  case State::INIT:
    init(byte);
    break;
  case State::FRAME:
    channel_decoder(byte);
    updateTimeout();
    // frame(byte);
    break;
  case State::INTER_FRAME:
    inter_frame(byte);
    break;
  /*case State::FUNCTION:
    function(data);
    break;*/
  case State::FAIL:
    // print_buf();
    setState(State::UNKNOWN);
    // case State::SYNC_LOST:
    //   find_sync(data);
    break;
  }
}