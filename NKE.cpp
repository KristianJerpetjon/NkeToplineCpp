#include "NKE.hpp"

uint16_t handle_angle(uint8_t *data){
        if (data[1] == '0x1') {
            return data[1]-100;
        }
        return data[1];
}

int16_t handle_int16(uint8_t *data) {
        //#a bit to lazy
        return (data[1]<<8 | data[0]);
}

bool nke::sync_sequence(uint8_t byte)
{
  //first one out is allways 02.. 00 is reserved to master and 01 is reserved for something else
  static uint8_t sequence=0x02;
  static uint8_t miss=0;
  static uint8_t curr=0xFF;

//lets pretend to be a slave

  if (byte==sequence )
  {
    if (sequence == SLAVE_ID && SLAVE_ENABLED)
    {
      // TODO write using ISR handler!
   //   Serial2.write(0x00);
   //   Serial2.write(0x00);
    }
    curr=sequence;
    sequence++;
    miss = 0;
  }
  else
  {
    miss+=1;
    if (miss == 2)
    {
      //printf("Detected sensor %02x\n",curr);
      m_sensors.push_back(curr);
    }

  }
  if (sequence > 238)
  {
    for(auto a :m_sensors )
    {
      printf("%02x ",a);
    }
    printf("\n");
    return false;
  }
  return true;
}

bool nke::handle_data(std::deque<uint8_t> &data)
{
  //to look for 73 1f 20 sequence.. 
  //we need to figure out if that sequence is horse shit 
  m_last_data.push_back(data[0]);
  if (m_last_data.size() > 3)
  {
    m_last_data.pop_front();
  }
  //TODO do something with the data!!
  //printf("data %02x,%02x,%02x\n",data[0],data[1],data[2]);
  //printf("data %02x\n",data[0]);
  if (data[0] == 0x1b)
  {
    //printf("time min=%d,sec=%d\n",data[1],data[2]);
  }
  data.clear();
  //if we have received 10 messages zeroize counter and return false
  return true;
}

bool nke::handle_controller(std::deque<uint8_t> &data)
{
  //TODO what if i am tze controller!!
    if (std::find(m_sync_pattern.begin(), m_sync_pattern.end(), data[0]) == m_sync_pattern.end())
    {
      //only print when not sync pattern
      printf("controller %02x,%02x,%02x",data[0],data[1],data[2]);
    }

    //this seems to work
    //seems FX commands should be received as 6 bytes or 5 after controller
    if (data[1] > 0xF0)
    {  
      m_decoder=Decoder::COMMAND;
    }
    data.clear();
    return true;
}

bool nke::handle_command(std::deque<uint8_t> &data)
{
  switch(data[0])
  {
    case 0x44:
      printf("Enter\n");
      break;
    case 0x42: //02 up/ 20 down
      switch(data[1])
      {
        case 0x02:
          printf("UP\n");
          break;
        case 0x20:
          printf("DOWN\n");
          break;
        break;
      }
      break;
    case 0x41:
      printf("Vertical Select\n");
      break;
    case 0x40:
      //data is encoded in controller as well..04 F4 01 / 02 /03 on each push
      //maybe 01 is "master" 02 first slave and 03 next.. should try out 
      //maybe we should deal with FX commands as 6 byte commands
      printf("Horizontal Select\n");
      break;
    default:
        printf("unknown cmd %02x,%02x,%02x\n",data[0],data[1],data[2]);
        break;
  };

  data.clear();
  return true;
}

void nke::update_expected(uint8_t controller)
{
    auto current=std::find(m_controllers.begin(), m_controllers.end(), controller);
      //need to asign next to m_next.
    if (current++ == m_controllers.end())
    {
        current=m_controllers.begin();
    }
    m_expected=*current;
}

bool nke::is_us()
{
  if (m_expected != m_us)
  {  
    return false;
  }
  
    //send either XX or send command
  printf("Send %02x\n",m_us);
  
  return true;
}

void nke::protocol_decoder(uint8_t byte)
{
  //const std::array<uint8_t,3> SYNC_PATTERN={0x1a,0xff,0xff};
  const std::deque<uint8_t> SYNC_PATTERN={0x1a,0xff,0xff};
  const std::deque<uint8_t> MSG_SEQUENCE={0x73,0x1f,0x20};
  //static std::array<uint8_t,3> rxbuf;

  static int handle_counter=0;
  static int inter_count=0;

  m_rxbuff.push_back(byte);

  switch(m_decoder)
  {
    case Decoder::NOSYNC:
      handle_counter=0;
      inter_count=0;
      if (m_rxbuff.size() > 3)
        m_rxbuff.pop_front();
      if (m_rxbuff == SYNC_PATTERN)
      {  
        //printf("sync\n");
        handle_data(m_rxbuff);
        handle_counter++;
        m_decoder=Decoder::DATA;
      }
      break;
    case Decoder::DATA:
      if (m_rxbuff.size() >  2)
      {
        handle_data(m_rxbuff);
        handle_counter++;
        if (handle_counter > 9){
          m_decoder=Decoder::SYNC;
          handle_counter=0;

          if (m_last_data != MSG_SEQUENCE )
          {
            //send if we should!
            is_us(); 
          }
          return;
        }
      }
      break;
      
      //if rxbuff[1]< 0x0F || > b0: default exit instead of counting!!
      //  sync!!
    case Decoder::SYNC:
    {
      if (inter_count > 1)
      {
        inter_count=0;
        m_decoder=Decoder::DATA;
        return ;
      }

      
          //std::list<int>::iterator findIter = std::find(ilist.begin(), ilist.end(), 1);
      if (m_last_data == MSG_SEQUENCE)
      //{
      //if (std::find(m_sync_pattern.begin(), m_sync_pattern.end(), rxbuff[0]) != m_sync_pattern.end())
      //#uint8_t controller=rxbuff[0];
      //bool pattern=false;
      //for(auto &a : m_sync_pattern)
      {
          if (m_rxbuff.size() > 2){
            handle_controller(m_rxbuff);
            m_last_data.clear();
            inter_count=1;
            //should we send now?
            is_us();
          }
          return;
      }
      //we dont expect data here to be in this range.. why are we not going back to syncing when we get this!
      if ((m_rxbuff[0] > 16) && (m_rxbuff[0] < 0xf0))
      { 
        printf("Error unpexpected command inter %d %02x\n",inter_count,m_rxbuff[0]);
        m_decoder=Decoder::NOSYNC;
        return;
      }

      if (std::find(m_controllers.begin(), m_controllers.end(), m_rxbuff[0]) == m_controllers.end())
      {
        m_controllers.push_back(m_rxbuff[0]);
        m_controllers.sort([](uint8_t a,uint8_t b) -> bool {
          return b > a;
        });
        printf("Controllers: ");
        for (auto &c : m_controllers){
          printf("%d ",c);
        }
        printf("\n");
      } /*else 
      {
        update_expected(rxbuff[0]);
        //asume complete_list
      }*/

      //We need to use timing instead of logic for this one
      //read out time between XX and XX++
      //printf("Sync \n",m_rxbuff.size());
      if (m_rxbuff.size() > 1)
      {
        if (m_rxbuff[0] == m_rxbuff[1] || m_rxbuff[1] >= 0xF0)
        {
          if (m_rxbuff.size() > 2)
          {
            handle_controller(m_rxbuff);
            inter_count++;
            if (inter_count==1)
            {
              is_us();
            }
          }
        }
        else
        {
          m_rxbuff.pop_front();
          inter_count++;
        }
      } 
      else //if we recieved only one slave and we are next.. send us 
      {
        //How do we know when to send!! this is a timing issue!
        if (inter_count==1)
        {
          is_us();
        }
      }
      break;
    }
    case Decoder::COMMAND:
      if (m_rxbuff.size() == 3){
        handle_command(m_rxbuff);
        m_decoder=Decoder::SYNC;
      }
      break;
    default:
      break;

  }
}