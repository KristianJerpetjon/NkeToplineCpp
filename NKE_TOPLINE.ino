#include "NKE.hpp"

#include <HardwareSerial.h>

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 (used in NMEA2000_CAN.H)
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include <NMEA2000_CAN.h>  // This will automatically choose the right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

//specify what serial port the NKE TOPLINE is connected to and what pins are in use for RX and TX
#define NKE_TOPLINE_SERIAL Serial2
#define NKE_TOPLINE_RXD 16
#define NKE_TOPLINE_TXD 17

#include "NKETopline.hpp"

#define SlowDataUpdatePeriod 1000  // Time between CAN Messages sent


const char *bah="Hello PC!!";


uint8_t request;
int resp_counter=0;
uint8_t response[2];


#define CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK

const int ledPin = 1;
static int pinState=LOW;


const unsigned long TransmitMessages0[] PROGMEM={130306L,0};

//TODO fetch second sensor from Boat to get the rest of these data bridged
const unsigned long TransmitMessages1[] PROGMEM={127250L,0};

// 
//how often does wind data get transmitted PGN has rules!
tN2kSyncScheduler WindScheduler(false,100,500);
tN2kSyncScheduler HeadingScheduler(false,100,600);


// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
  //TODO figure out a better way to schedule multiple messages..
  WindScheduler.UpdateNextTime();
  HeadingScheduler.UpdateNextTime();
}

double heading=0;

nke Nke;


static NkeData value;

void setup() {
  
  // Initialize serial
  //Serial.begin(115200);
  Serial.begin(921600);
  Serial.write(bah,strlen(bah));

/*
  Nke.add_decoder(0x16, [](uint8_t request,uint8_t* response){
    printf("%02x: %02x %02x\n",request,response[0],response[1]);
  });*/

  //what does the slave push out ? .. it pushes more than one thing
  Nke.add_decoder(0x03, [](uint8_t request,uint8_t* response){
    //printf("Slave %02x %02x\n",response[0],response[1]);
  });

  Nke.add_decoder(0x18, [](uint8_t request,uint8_t* response){
    int16_t angle=(((response[0]&0xff)<<8 ) | (response[1] & 0xff));

    //printf("0x18 %02x %02x\n",response[0],response[1]);

    //int16_t awd=(((response[0]&0xff)<<8 ) | (response[1] & 0xff));
    int deg=int(angle/3.141592);
    //printf("aparent_wind_angle 0x18 %d\n",deg);
  });

  Nke.add_decoder(0x1b, [](uint8_t request,uint8_t* response){
    //second counter for what.. ? 
    int16_t value_int=(((response[0]&0xff)<<8 ) | (response[1] & 0xff));
  //  printf("Request %02x %02x %02x %d\n",request,response[0],response[1],value_int);
  });

  Nke.add_decoder(0x30, [](uint8_t request,uint8_t* response){
    int16_t value_int=(((response[0]&0xff)<<8 ) | (response[1] & 0xff));
    printf("Request %02x %02x %02x %d\n",request,response[0],response[1],value_int);
  });
  
  Nke.add_decoder(0x3b, [](uint8_t request,uint8_t* response){
    int16_t value_int=(((response[0]&0xff)<<8 ) | (response[1] & 0xff));
    printf("Request %02x %02x %02x %d\n",request,response[0],response[1],value_int);
  });
  /*Nke.add_decoder(0x19, [](uint8_t request,uint8_t* response){
    printf("0x19 %02x %02x\n",response[0],response[1]);
  });*/


  Nke.add_decoder(0x19, [](uint8_t request,uint8_t* response){
    //printf("Raw %02x %02x\n",response[0],response[1]);

    int16_t heading=(((response[0]&0xff)<<8 ) | (response[1] & 0xff));
    printf("Heading %d\n",heading);
  });


  /*Nke.add_decoder(0x73, [](uint8_t request,uint8_t* response){
    printf("Trim %d\n",handle_angle(response));
  });*/

  //Get Unique ID from mac addres in efuse..
  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0;
  esp_efuse_mac_get_default(chipid);
    for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANMsgBufSize(8);

  //we are emulating two devices
  NMEA2000.SetDeviceCount(2);

   // Set Product information
  NMEA2000.SetProductInformation("00000002", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Simple wind monitor",  // Manufacturer's Model ID
                                 "1.2.0.24 (2022-10-01)",  // Manufacturer's Software version code
                                 "1.2.0.0 (2022-10-01)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 0 ///dev id
                                 );

  NMEA2000.SetProductInformation("00000002", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Nke Heading",  // Manufacturer's Model ID
                                 "1.2.0.24 (2022-10-01)",  // Manufacturer's Software version code
                                 "1.2.0.0 (2022-10-01)", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 1 ///dev id
                                 );
  //probably not so important.. maybe
  // Set device information https://manualzz.com/doc/12647142/nmea2000-class-and-function-codes
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number.
                                150, //Device function Bridge 
                                25, // intranetwork device

                                //130, // Device function=Atmospheric. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                //85, // Device class=External Environment. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046, // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4, // Marine
                                0 //dev id
                               );

  NMEA2000.SetDeviceInformation(id+1, // Unique number. Use e.g. Serial number.
                                150, //Device function Ownship Attitude Heading ,pitch, roll,yaw, angular rates 
                                60, // intranetwork device

                                //130, // Device function=Atmospheric. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                //85, // Device class=External Environment. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046, // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4, // Marine
                                1 //dev id
                               );
  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader                           
  //Serial.begin(115200);
  //NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,23);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(false);
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages0,0);
  NMEA2000.ExtendTransmitMessages(TransmitMessages1,1);

  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();

  class NkeCompassHandler : public NkeHandler 
  {
    public:
    NkeCompassHandler(std::function<void(double)> func=nullptr) 
     : NkeHandler(0x19) 
     , m_func(func)
     { }

    void handle(const NkeMessage &msg) override 
    {
      //Serial.printf("Compass %02x values %02x %02x\n",msg.cmd,msg.data[0],msg.data[1]);
      if (m_func) {
        uint16_t heading=msg.data[0]<<8|msg.data[1]&0xff;
        const double pi=3.141592;
        double rad=((heading*pi)/180.0);
        m_func(rad);
      }
    }
    private:
    std::function<void(double)> m_func;
  };



  //NkeCompassHandler compassHandler;
  auto compassHandler=std::make_shared<NkeCompassHandler>([&heading](double head){
    heading=head;
  });
  NkeTopline.AddHandler(compassHandler);
  
  class GenericHandler : public NkeHandler
  {
    public: 
    GenericHandler(uint8_t stubid,uint16_t data=0x00) //std::function<void(uint8_t data*)> func = nullptr)
    : NkeHandler(stubid)
    {
      m_data.data[0]=(data>>8)&0xff;
      m_data.data[1]=data&0xff;
    }

    void setData(uint8_t *data)
    {
      memcpy(m_data.data,data,2);
    }

  void handle(const NkeMessage &msg) override 
  {
    
      Serial.printf("Generic value %02x values %02x %02x\n",msg.cmd,msg.data[0],msg.data[1]);
      /*
      if (m_func) {
        uint16_t heading=msg.data[0]<<8|msg.data[1]&0xff;
        const double pi=3.141592;
        double rad=((heading*pi)/180.0);
        m_func(rad);
      }*/
      uint16_t data=msg.data[0]<<8|msg.data[1]&0xff;
      data++;
      m_data.data[0]=(data>>8)&0xFF;
      m_data.data[1]=data&0xFF;

    }

    NkeData *data() 
    {
      return &m_data;
    }
    private:
    NkeData m_data;
    //todo make a lambda for "manipulating data"
    //std::function<
  };

  auto genericHandler19=std::make_shared<GenericHandler>(0x19);
  auto genericHandler3d=std::make_shared<GenericHandler>(0x3d);

  //recieve and send genericHandler
  //NkeTopline.AddHandler(genericHandler19);
  //NkeTopline.AddHandler(genericHandler3d);

  //Findings 
  // 18 fast wind speed? TODO check (fast is 1X so probably)
  // 3B app wind speed slow
  // asuming 18 is fast wind speed.

  //fc command is for device.. so fc 3b write 1 byte to reg 1b ? fc  3b 01 1b ff //1b should ack this command with 0x00 it seems.. so now we know command structure.. 
  //FC -> Dest -> reg + data ? .. ack with 00 ? 

  auto genericHandler3b=std::make_shared<GenericHandler>(0x3b,0x20);
  NkeTopline.addSendData(genericHandler3b->id(),genericHandler3b->data());

  // 0x3c aparent wind angle!
  auto genericHandler3c=std::make_shared<GenericHandler>(0x3c,0x10);
  NkeTopline.addSendData(genericHandler3c->id(),genericHandler3c->data());

  //auto genericHandler3d=std::make_shared<GenericHandler>(0x3d);

  //NkeTopline.addSendData(genericHandler->id(),genericHandler->data());
  //NkeTopline.addSendData(genericHandler->id(),genericHandler->data());


  NkeTopline.Open();
  //start listening on serial port 2 for NKE data
    //9600 is more or les 1 character per millisecond what should the timeout be
  //TODO remember that false/true here causes havoc old / new design old is true new is false
  //Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2,false,2);    //Hardware Serial of ESP32
  //Serial2.setRxTimeout(1);
  //Serial2.onReceive([&Serial2,&Nke](void){

    
    /*while(Serial2.available())
    {
      auto data=(uint8_t)Serial2.read();
      printf("bytes %02x\n",data);

      //reads++;
    }*/
    //if (reads > 1)
  
    //size_t available = Serial2.available();
    //if (available > 1)
      //printf("available %d\n",available);
//
//    while (available-- > 0) {
 //     auto data=(uint8_t)Serial2.read();
      //ok so this is to slow/deep !! what to do 
      //Nke.protocol_decoder(data);
  //});

}

void loop() {


  SendN2kWind();

  //TODO 
  //SetN2kRudder send rudder info to nmea2k
  SendN2kHeading();
  NMEA2000.ParseMessages();
  NkeTopline.ParseMessages();
}


// *****************************************************************************
double ReadWindAngle(bool update) {
  static int deg=0;
  static bool dir=false;
  double ret=0;
  const double pi=3.141592;
  //0-2PI
  int deg_corr=deg;
  if (deg_corr < 0)
    deg_corr=360-deg;
  deg_corr=deg%360;

  ret=((deg_corr*pi)/180);
  //ret=DegToRad(deg);
  if (update)
  {
    if (dir)
      deg--;
    else
      deg++;

    if (deg <= -180)
      //deg+=360
      dir=false;
    if (deg >= 180)
      dir=true;
  }
  return ret;
  //return DegToRad(50); // Read here the measured wind angle e.g. from analog input
}

// *****************************************************************************
double ReadWindSpeed(bool update) {
  //Fake speed in knots.. 

  static double windspeed=0.0;
  static bool dir=true;
  //for (int i = 0; i < 20;i++)
  //{/'
  if (update)
  {
    if (windspeed > 20.0)
      dir=false;
    if (windspeed < 1.0) 
      dir=true;

    if (dir)
      windspeed = windspeed+0.5;
    else
      windspeed = windspeed - 0.5;
  }
  //}

  return windspeed; // Read here the wind speed e.g. from analog input
}

// *****************************************************************************
void SendN2kWind() {
  tN2kMsg N2kMsg;
  static int count=0;
  if ( WindScheduler.IsTime() ) {
    //printf("Sending Wind\n");
    count++;
    bool update=false;

    if (count > 10)
    {
      count=0;
      update=true;
    }

    WindScheduler.UpdateNextTime();
    uint8_t sid=255;
    SetN2kWindSpeed(N2kMsg, sid, ReadWindSpeed(update), ReadWindAngle(update),N2kWind_Apprent);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kHeading()
{
  tN2kMsg N2kMsg;
  if ( HeadingScheduler.IsTime() ) {
    HeadingScheduler.UpdateNextTime();
    uint8_t sid=255;
    SetN2kMagneticHeading(N2kMsg,sid,heading);
    NMEA2000.SendMsg(N2kMsg);
  }


  //SetN2kTrueHeading(tN2kMsg &N2kMsg, unsigned char SID, double Heading)
}

