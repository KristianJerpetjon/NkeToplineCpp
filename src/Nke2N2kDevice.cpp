// #include "NKE.hpp"
// #include <M5Tough.h>

// #include <HardwareSerial.h>
/*
#define ESP32_CAN_TX_PIN GPIO_NUM_5 // Set CAN TX port to 5 (used in NMEA2000_CAN.H)
#define ESP32_CAN_RX_PIN GPIO_NUM_4 // Set CAN RX port to 4
*/

#include <M5Unified.h>

#define ESP32_CAN_TX_PIN GPIO_NUM_27 // Set CAN TX port to 5 (used in NMEA2000_CAN.H)
#define ESP32_CAN_RX_PIN GPIO_NUM_19 // Set CAN RX port to 4

#include <NMEA2000_CAN.h> // This will automatically choose the right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>
// specify what serial port the NKE TOPLINE is connected to and what pins are in use for RX and TX
#define NKE_TOPLINE_SERIAL Serial2

#define NKE_TOPLINE_RXD GPIO_NUM_13
#define NKE_TOPLINE_TXD GPIO_NUM_14

// This is just so not nice!
#include "include/NKETopline.hpp"
#include "include/NkeAutopilot.hpp"
#include "include/NavicoAp.hpp"
#include "include/N2KNkeHandler.hpp"

uint32_t count;
// need to toggle some gpios
// #include "driver/gpio.h"

#include <Preferences.h>
#include <memory>
#include <list>

#define SlowDataUpdatePeriod 1000 // Time between CAN Messages sent
Preferences preferences;          // Nonvolatile storage on ESP32 - To store LastDeviceAddress

int NodeAddress; // To store last Node Address

const char *bah = "Hello PC!!";

uint8_t request;
int resp_counter = 0;
uint8_t response[2];

#define CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK

const int ledPin = 1;
static int pinState = LOW;
int LED_BUILTIN = 2;

const unsigned long TransmitMessages0[] PROGMEM = {
    130306L,
    127250L,
    127245L, // RUDDER priority 2 period 100ms
    0};

const unsigned long ReceiveMessages0[] PROGMEM = {/*126992L,*/ // System time
                                                               // 127250L, // Heading
                                                               // 127258L, // Magnetic variation
                                                  128259L,     // Boat speed
                                                  128267L,     // Depth
                                                  // 129025UL,// Position
                                                  // 129026L, // COG and SOG
                                                  // 129029L, // GNSS
                                                  // 130306L, // Wind
                                                  // DST800 Sends
                                                  //  128259UL for speed
                                                  //  128267UL for depth
                                                  //  1282765UL for distance log
                                                  130310L, // deprecated temp legacy
                                                  // 130311L,
                                                  // 130310,130311,130312 for temoerature
                                                  0};

/*

*/
/*
DST810 for reference
059904..............ISO Request
060928..............ISO Address Claim
065240..............Commanded Address
59392................ISO Acknowledgement
126208..............Acknowledge Group Function
126464..............Transmit PGN List Group Function
126464..............Received PGN List Group Function
126993..............Heartbeat
126996..............Product Information
126998..............Configuration Information
127257..............Attitude
128259..............Speed (Speed Water Reference)
128267..............Water Depth (With Transducer Offset)
128275..............Distance Log
130310..............Environmental Parameters (Water Temperature)
130311..............Environmental Parameters (Water Temperature)
130312..............Environmental Parameters (Water Temperature)
130316..............Temperature, Extended Range

*/

/*
tNMEA2000Handler NMEA2000Handlers[]={
{126992L,&SystemTime},
{127245L,&Rudder },
{127250L,&Heading},
{127257L,&Attitude},
{127488L,&EngineRapid},
{127489L,&EngineDynamicParameters},
{127493L,&TransmissionParameters},
{127497L,&TripFuelConsumption},
{127501L,&BinaryStatus},
{127505L,&FluidLevel},
{127506L,&DCStatus},
{127513L,&BatteryConfigurationStatus},
{128259L,&Speed},
{128267L,&WaterDepth},
{129026L,&COGSOG},
{129029L,&GNSS},
{129033L,&LocalOffset},
{129045L,&UserDatumSettings},
{129540L,&GNSSSatsInView},
//130306 ? wind data!!
{130310L,&OutsideEnvironmental},
{130312L,&Temperature},
{130313L,&Humidity},
{130314L,&Pressure},
{130316L,&TemperatureExt},
{0,0}
};*/
#include "Nke/include/Display.hpp"

// TODO move M5 code to separate files
std::unique_ptr<Nke::Display> nkedisplay;
void setupM5()
{
    auto cfg = M5.config(); // Assign a structure for initializing M5Stack
    // If config is to be set, set it here
    // Example.
    // cfg.external_spk = true;
    cfg.output_power = false;

    // lets see if we can play sound!!

    cfg.external_speaker.module_display = true;

    M5.begin(cfg); // initialize M5 device

    nkedisplay = std::unique_ptr<Nke::Display>(new Nke::Display(M5.Display, "app wind a"));
    // M5.Display.setTextSize(7); // change text size
    //(135*240)
    //  M5.Display.drawRect(0, 0, 420, 40, 0xff80);
    //  M5.Display.drawRect(0, 41, 320, 240, 0x001F);
    //  M5.Display.print("Hello World!!!"); // display Hello World! and one line is displayed on the screen
    //  Serial.println("Hello World!!!");   // display Hello World! and one line on the serial monitor
    count = 0; // initialize count

    // plays a tone on startup we should play a tone on NKE connected and disconnected ? smae with CAN ?
    M5.Speaker.setVolume(64);
    M5.Speaker.tone(1000, 1000, 0, true);

    // M5.Speaker.setBeep(1000,1000);
    // M5.Speaker.beep();
}

// TODO fetch second sensor from Boat to get the rest of these data bridged
const unsigned long TransmitMessages1[] PROGMEM = {127250L, 0};

//
// how often does wind data get transmitted PGN has rules!
tN2kSyncScheduler WindScheduler(false, 100, 500);
tN2kSyncScheduler HeadingScheduler(false, 100, 600);
tN2kSyncScheduler RudderScheduler(false, 100, 50);

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen()
{
    // Start schedulers now.
    // TODO figure out a better way to schedule multiple messages..
    WindScheduler.UpdateNextTime();
    HeadingScheduler.UpdateNextTime();
    RudderScheduler.UpdateNextTime();
}

double heading = 0;

// TouchZone topHalf(0, 0, 320, 120);
// TouchZone bottomHalf(0, 120, 320, 160);
//  Gesture swipeDown(topHalf, bottomHalf, "Swipe Down");
//   swipeUp(bottomHalf, topHalf, "Swipe Up");

// nke Nke;

// static NkeData value;
// NkeDevice
// static std::map<unsigned long, std::shared_ptr<NkeDevice>> N2KHandlers;

#include <unordered_map>

std::unique_ptr<NavicoAp> m_navico;

NkeBridge bridge;

// map for handlers of NkeDevices
static const std::unordered_map<unsigned long, std::shared_ptr<N2kNkeHandler>> N2kHandlers = {
    {128259L, std::make_shared<HandleBoatSpeed>(bridge)},
    {130310L, std::make_shared<HandleWaterTemp>(bridge)},
    {130306L, std::make_shared<HandleWindSpeed>(bridge)},
    {129025L, std::make_shared<HandlePosition>(bridge)},
    {129026L, std::make_shared<HandleCogSog>(bridge)}};

// need to look at unordered map for fast pgn lookup
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
    // Serial.printf("Handled msg to %u\n",N2kMsg.PGN);

    auto idx = N2kHandlers.find(N2kMsg.PGN);
    if (idx != N2kHandlers.end())
    {
        digitalWrite(32, HIGH);

        idx->second->handleN2kData(N2kMsg);
        digitalWrite(32, LOW);

        // Serial.printf("Msg to %u Handled\n",N2kMsg.PGN);

        // dev->handleN2kData(N2kMsg);
    }

    if (m_navico != nullptr)
    {
        m_navico->HandleNMEA2000Msg(N2kMsg);
    }
}
TaskHandle_t TaskNke;
TaskHandle_t TaskN2k;
TaskHandle_t TaskAp;

std::unique_ptr<AutopilotController> m_auto;

int m_id = 1;

tNKETopline NkeTopline(Serial2, NKE_TOPLINE_RXD, NKE_TOPLINE_TXD);

// *****************************************************************************
double ReadWindSpeed(bool update)
{
    // Fake speed in knots..

    static double windspeed = 0.0;
    static bool dir = true;
    // for (int i = 0; i < 20;i++)
    //{/'
    if (update)
    {
        if (windspeed > 20.0)
            dir = false;
        if (windspeed < 1.0)
            dir = true;

        if (dir)
            windspeed = windspeed + 0.5;
        else
            windspeed = windspeed - 0.5;
    }
    //}

    return windspeed; // Read here the wind speed e.g. from analog input
}

double ReadWindAngle(bool update)
{
    static int deg = 0;
    static bool dir = false;
    double ret = 0;
    const double pi = 3.141592;
    // 0-2PI
    int deg_corr = deg;
    if (deg_corr < 0)
        deg_corr = 360 - deg;
    deg_corr = deg % 360;

    ret = ((deg_corr * pi) / 180);
    // ret=DegToRad(deg);
    if (update)
    {
        if (dir)
            deg--;
        else
            deg++;

        if (deg <= -180)
            // deg+=360
            dir = false;
        if (deg >= 180)
            dir = true;
    }
    return ret;
    // return DegToRad(50); // Read here the measured wind angle e.g. from analog input
}

// *****************************************************************************
void SendN2kWind()
{
    tN2kMsg N2kMsg;
    static int count = 0;
    if (WindScheduler.IsTime())
    {
        // Serial.printf("Sending Wind\n");
        count++;
        bool update = false;

        if (count > 10)
        {
            count = 0;
            update = true;
        }

        WindScheduler.UpdateNextTime();
        uint8_t sid = 255;
        SetN2kWindSpeed(N2kMsg, sid, ReadWindSpeed(update), ReadWindAngle(update), N2kWind_Apprent);
        NMEA2000.SendMsg(N2kMsg, m_id);
    }
}

void SendN2kHeading()
{
    tN2kMsg N2kMsg;
    if (HeadingScheduler.IsTime())
    {
        HeadingScheduler.UpdateNextTime();
        uint8_t sid = 255;
        SetN2kMagneticHeading(N2kMsg, sid, bridge.heading().N2k());
        NMEA2000.SendMsg(N2kMsg, m_id);
    }
}

void SendN2kRudder()
{
    tN2kMsg N2kMsg;
    static int count = 0;
    if (RudderScheduler.IsTime())
    {
        SetN2kRudder(N2kMsg, bridge.rudderAngle().N2k());
        NMEA2000.SendMsg(N2kMsg, m_id);
        RudderScheduler.UpdateNextTime();
    }
}
void SendN2kMessages()
{
    SendN2kRudder();
    SendN2kHeading();
}

void N2KRun()
{
    // SendN2kWind();
    //  TODO
    //  SetN2kRudder send rudder info to nmea2k
    // SendN2kHeading(); //For some reason.. sending fails!!
    SendN2kMessages();
    NMEA2000.ParseMessages();
    if (m_navico != nullptr)
    {
        m_navico->process();
    }

    int SourceAddress = NMEA2000.GetN2kSource(m_id);
    if (SourceAddress != NodeAddress)
    { // Save potentially changed Source Address to NVS memory
        /*NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
        preferences.begin("nvs", false);
        preferences.putInt("LastNodeAddress", SourceAddress);
        preferences.end();
        logger.logDebug(GwLog::LOG,"Address Change: New Address=%d\n", SourceAddress);*/
        Serial.printf("SourceAddress changed to %d\n", SourceAddress);
        NodeAddress = SourceAddress;
    }
}

// need to move the senders to a class
#include <sstream>
#include <iomanip>

void ApLoop(void *pvParameters)
{
    unsigned long update = millis() + 1000;
    unsigned long touch_update = millis() + 4;
    uint8_t filter = 0xFF;
    NkeTopline.addMsgHandler([&filter](const Nke::_Message &msg)
                             {
                    if (msg.channel == filter) {
                    if (msg.len==2) {
                        uint8_t hi=msg.data[0];
                        uint8_t low=msg.data[1];
                        uint16_t val=hi<<8|low;
                        Serial.printf("Received channel %02x(%d) %02x(%d) %02x(%d) val=%d\n",filter,filter,hi,hi,low,low,val);
                        //uint16_t data=msg.data[0]<<8|msg.data[1];
                        //bridge.setHeading(data);
                        }
                    } });

    for (;;)
    {
        unsigned long now = millis();
        if (now > update)
        {
            update = now + 33; // 30fps update
            // update = now + 1000;
            char buf[20] = "";

            auto deg = RadToDeg(bridge.windAngle().N2k());
            deg = floor(deg);
            if (deg > 180)
                deg = -(360 - deg);
            // ss << deg << "º";

            int res = static_cast<int>(deg);

            snprintf(buf, sizeof(buf), "%d%c", res, 247);
            // std::stringstream ss;
            // ss << std::setw(2);
            // ss << std::setprecision(1); // rounds to std::out
            // auto deg = RadToDeg(bridge.windAngle().N2k());
            // deg = std::round(deg / 0.1) * 0.1;
            // if (deg > 180)
            //    deg = -(360 - deg);
            // ss << deg << "º";

            // nkedisplay->update(ss.str().c_str());
            nkedisplay->update(buf);

            /*M5.Display.setTextSize(5);
            M5.Display.setCursor(0, 3); // set character drawing coordinates (cursor position)
            // M5.Display.printf("COUNT: %d\n", count); // display count on screen
            M5.Display.printf("Topline RX %u\n", NkeTopline.receiveCount());
            M5.Display.printf("Topline TX %u\n", NkeTopline.sendCount());
*/
            // Serial.printf("COUNT: %d\n", count);  // display count serially

            count++; // increase count by 1
        }

        if (now > touch_update)
        {
            touch_update = now + 4; // something like this
            // M5.update();
            auto touch_count = M5.Touch.getCount();
            static m5::touch_state_t prev_state;
            // Gesture swipeDown(topHalf, bottomHalf, "Swipe Down");
            // m5::Button_Class::Gesture

            if (touch_count > 0)
            {
                auto t = M5.Touch.getDetail();
                if (prev_state != t.state)
                {
                    prev_state = t.state;
                    static constexpr const char *state_name[16] =
                        {"none", "touch", "touch_end", "touch_begin", "___", "hold", "hold_end", "hold_begin", "___", "flick", "flick_end", "flick_begin", "___", "drag", "drag_end", "drag_begin"};
                    // M5_LOGI("%s", state_name[t.state]);
                    Serial.printf("Touch %s\n", state_name[t.state]);

                    // maybe we need to run the touch stuff at > frequency than the disp
                    /*
                        static constexpr std::size_t TOUCH_MAX_POINTS = 3;
                        static constexpr std::size_t TOUCH_MIN_UPDATE_MSEC = 4;
                    */
                    if (t.state == m5::touch_state_t::touch_begin)
                    {
                    }

                    if (t.state == 2)
                    { // touch end..
                        // detect swipe using simple swipe algorithm where did it begin.. and where did it end
                        printf("Delta X %d\n", t.distanceX());
                        printf("Delta Y %d\n", t.distanceY());
                    }
                }
            }
        }

        if (Serial.available() > 0)
        {
            String data = Serial.readStringUntil('\n');

            if (data.startsWith("debug"))
            {

                auto s = data.substring(6);
                filter = s.toInt();
                Serial.printf("Debugging %02x\n", filter);
                // NkeTopline.addHandler(compassHandler);
                /*NkeTopline.addMsgHandler([bridge](const Nke::_Message &msg)
                                         {
                    if (msg.channel == s.toInt()) {
                    if (msg.len==2) {
                        Serial.printf("Received channel %d")
                        //uint16_t data=msg.data[0]<<8|msg.data[1];
                        //bridge.setHeading(data);
                        }
                    } });*/

                // NkeTopline.addHandler(st::make_shared<NkeHandler>({}));
            }
            if (data == "+1")
            {
                m_auto->change(1);
            }

            if (data == "+10")
            {
                m_auto->change(10);
            }

            if (data == "-1")
            {
                m_auto->change(-1);
            }

            if (data == "-10")
            {
                m_auto->change(-10);
            }

            if (data == "ON")
            {
                // send autopilot on
                if (m_auto)
                {
                    m_auto->start();
                }
            }

            if (data == "OFF")
            {
                // send autopilot off
                if (m_auto)
                {
                    m_auto->stop();
                }
            }
            if (data == "RUDDER")
            {
                // we can have autopilots on both 4E and 4F .. afaik for redundancy
                if (m_auto)
                {
                    m_auto->setPilotMode(PilotMode::Rudder);
                }

                // Serial.printf("Sending Rudder to NkeTopline\n");
                // sendCommand(0xf4,0x4f,0x1,{0x00,0x02});
                // sendCommand(0xfc,0x4f,0x1,{});
                /*tNkeMsg msg;
                msg.channel=0xf4;
                //was 4e the other controller ?

                //send write command to f4 to write into reg 01 the value 00 02 (02 is rudder mode) TODO move this to an autopilot controller class
                msg.data={0x4f,0x01,00,02};
                msg.len=4;
                NkeTopline.sendCommand(msg);
                //perform read back
                msg.channel=0xfc;
                msg.data={0x4f,0x01};
                msg.len=2;
                NkeTopline.sendCommand(msg);*/
            }
            if (data == "COMPASS")
            {
                if (m_auto)
                {
                    m_auto->setPilotMode(PilotMode::Compass);
                }
                // sendCommand(0xf4,0x4f,0x1,{0x00,0x00});
                // sendCommand(0xfc,0x4f,0x1,{});
            }

            if (data == "APARENT")
            {
                if (m_auto)
                {
                    m_auto->setPilotMode(PilotMode::Aparent);
                }
                // why does the system allways read 4e reg 10 before setting mode..
                // sendCommand(0xf4,0x4f,0x1,{0x00,0x01});
                // sendCommand(0xfc,0x4f,0x1,{});
            }
        }
    }
    vTaskDelay(1);
    taskYIELD();
}

void N2Kloop(void *pvParameters)
{
    m_navico->start();
    for (;;)
    {
        N2KRun();
        vTaskDelay(1);
        taskYIELD();
    }
}

void setup()
{

    setupM5();

    pinMode(32, OUTPUT); // set the pin as output
    digitalWrite(32, HIGH);
    pinMode(33, OUTPUT); // set the pin as output
    digitalWrite(33, HIGH);

    // todo automate multicore!

    // Initialize serial
    // Serial.begin(115200);
    Serial.begin(921600);
    // Serial.write(bah, strlen(bah));'
    Serial.printf("Nke N2k Bridge Version %d,%d\n", 0, 1);

    Serial.print("setup() running on core ");
    Serial.println(xPortGetCoreID());

    // Get Unique ID from mac addres in efuse..
    uint8_t chipid[6];
    uint32_t id = 0;
    int i = 0;
    esp_efuse_mac_get_default(chipid);
    for (i = 0; i < 6; i++)
        id += (chipid[i] << (7 * i));

    NMEA2000.SetN2kCANMsgBufSize(8);
    // NMEA2000.SetN2kCANReceiveFrameBufSize(150);
    // mighgt be uneccesary amount
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);

    // we are emulating two devices
    // second device is the m_navico_ap
    NMEA2000.SetDeviceCount(2);

    // device id here

    // Set Product information
    NMEA2000.SetProductInformation("00000002",              // Manufacturer's Model serial code
                                   100,                     // Manufacturer's product code
                                   "NKE N2K Bridge",        // Manufacturer's Model ID
                                   "1.2.0.24 (2022-10-01)", // Manufacturer's Software version code
                                   "1.2.0.0 (2022-10-01)",  // Manufacturer's Model version
                                   0xff,                    // load equivalency - use default
                                   0xffff,                  // NMEA 2000 version - use default
                                   0xff,                    // Sertification level - use default
                                   m_id                     /// dev id
    );

    NMEA2000.SetDeviceInformation(id,  // Unique number. Use e.g. Serial number.
                                  150, // intranetwork device 130// Device function Ownship Attitude Heading ,pitch, roll,yaw, angular rates
                                  60,  // intranetwork device

                                  // 130, // Device function=Atmospheric. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  // 85, // Device class=External Environment. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  2046, // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                  4,    // Marine
                                  m_id  // dev id
    );

    // Maybe multi device is broken ?
    NMEA2000.ExtendTransmitMessages(TransmitMessages0, m_id);
    NMEA2000.ExtendReceiveMessages(ReceiveMessages0, m_id);

    m_navico = std::unique_ptr<NavicoAp>(new NavicoAp(NMEA2000, bridge, id + 1, 0));

    // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader
    // Serial.begin(115200);
    // NMEA2000.SetForwardStream(&Serial);
    // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
    // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

    preferences.begin("nvs", false);                         // Open nonvolatile storage (nvs)
    NodeAddress = preferences.getInt("LastNodeAddress", 34); // Read stored last NodeAddress, default 34
    preferences.end();
    Serial.printf("NodeAddress=%d\n", NodeAddress);

    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    // NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);
    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 32);
    // N2km_ListenAndNode
    //  NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
    NMEA2000.EnableForward(false);
    // Here we tell library, which PGNs we transmit

    // this attaches a parser class object
    // NMEA2000.AttachMsgHandler(&HandleNMEA2000Msg);
    //  We can attach two msg handlers!!

    // this adds a callback
    NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
    // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.

    class NkeCompassHandler : public NkeHandler
    {
    public:
        NkeCompassHandler(std::function<void(double)> func = nullptr)
            : NkeHandler(0x19), m_func(func)
        {
        }

        void handle(const tNkeMsg &msg /*const uint16_t &heading*/) override
        {
            // Serial.printf("Compass %02x values %02x %02x\n",msg.cmd,msg.data[0],msg.data[1]);
            uint16_t heading = msg.data[0] << 8 | msg.data[1];
            if (m_func)
            {

                const double pi = 3.141592;
                double rad = ((heading * pi) / 180.0);
                m_func(rad);
            }
        }

    private:
        std::function<void(double)> m_func;
    };

    // NkeCompassHandler compassHandler;
    auto compassHandler = std::make_shared<NkeCompassHandler>([](double head)
                                                              { heading = head; });

    // NkeTopline.addHandler(compassHandler);
    NkeTopline.addMsgHandler([bridge](const Nke::_Message &msg)
                             {
    if (msg.channel == (uint8_t)Nke::Channel::HEADING_FAST/*0x19*/) {
       if (msg.len==2) {
          uint16_t data=msg.data[0]<<8|msg.data[1];
          bridge.setHeading(data);
        }
      } });

    NkeTopline.addMsgHandler([bridge](const Nke::_Message &msg)
                             {
    //17 is fixed at 180.. i wonder what that is.. 
    //1e is rudder fast where 0x4xx means negative and 0xx means positive.. where 0x434 is close to zero .. and 400 is not. 
    //need to find the decoding as the autopilot expects 359 as negative and 1 as positive integer.
    if (msg.channel == (uint8_t)Nke::Channel::RUDDER_FAST) { //asume its rudder fast
      if (msg.len==2)
      {
        uint16_t data=msg.data[0]<<8|msg.data[1];
        //bridge.setRunnerAngleNke(data);
        bridge.setRunnerAngle(data);
        /*
        if (data > (180*3))
        {
          int computed=(data/3);
          computed=-(360-computed);
          Serial.printf("Rudder Angle %d\n",computed);

        } else {
          Serial.printf("Rudder Angle %d\n",(data/3));
        }*/
      }
    } });

    NkeTopline.addMsgHandler([&bridge](const Nke::_Message &msg)
                             {
    if (msg.channel == 0x4e) { //asume its rudder fast
      if (msg.len==2) {
        //Serial.printf("Pilot %02x,%02x,%02x\n",msg.channel,msg.data[0],msg.data[1]);
        //constantly 01,66 ? why ? 
      }
    } });

    NkeTopline.addMsgHandler([&bridge](const Nke::_Message &msg)
                             {
      //when active shows value of reg 11 if not FF (probably means no value or issued by master!)
    if (msg.channel == 0x4f) { //asume its rudder fast
      if (msg.len==2) {
        //Serial.printf("Pilot %02x,%02x,%02x\n",msg.channel,msg.data[0],msg.data[1]);
      }
    } });

    NkeTopline.addMsgHandler([&bridge](const Nke::_Message &msg)
                             {
    if (msg.channel == 0x41) { //asume its rudder fast
      if (msg.len==2) {
        //constantly FF at least in compass mode
        //Serial.printf("Pilot %02x,%02x,%02x\n",msg.channel,msg.data[0],msg.data[1]);
      }
    } });

    NkeTopline.addMsgHandler([&bridge](const Nke::_Message &msg)
                             {
      //when active shows value of reg 11 if not FF (probably means no value or issued by master!)
    if (msg.channel == 0x50) { //asume its rudder fast
      if (msg.len==2) {
        //constantly FF at least in compass mode
//        Serial.printf("Pilot %02x,%02x,%02x\n",msg.channel,msg.data[0],msg.data[1]);
      }
    } });

    // shows up as id 2!!
    //  probably better if nkedata is default in nke handler.. also we need to deal with multiple!!

    /*
    auto genericHandler19=std::make_shared<GenericHandler>(0x19);
    auto genericHandler3d=std::make_shared<GenericHandler>(0x3d);
    auto genericHandler3d=std::make_shared<GenericHandler>(0x30);
  */

    // recieve and send genericHandler
    // NkeTopline.AddHandler(genericHandler19);
    // NkeTopline.AddHandler(genericHandler3d);

    // Findings
    //  18 fast wind speed? TODO check (fast is 1X so probably)
    //  3B app wind speed slow
    //  asuming 18 is fast wind speed.

    // fc command is for device.. so fc 3b write 1 byte to reg 1b ? fc  3b 01 1b ff //1b should ack this command with 0x00 it seems.. so now we know command structure..
    // FC -> Dest -> reg + data ? .. ack with 00 ?

    // auto genericHandler3b=std::make_shared<GenericHandler>(0x3b,0x20);
    // TEMP
    /*
      auto handler30 = std::make_shared<GenericHandler>(0x30, 0x69);
      NkeTopline.addSendData(handler30->id(), handler30->data());
      handlers.push_back(handler30);
    */
    NkeTopline.addDevice(std::make_shared<NkeDevice>(0x30, 1, 9));

    NkeTopline.addDevice(std::make_shared<NkeDevice>(0x02, 5, 5));

    auto speed = std::make_shared<Nke::BoatSpeed>(bridge);
    // NkeTopline.addDevice(std::make_shared<Nke::Speed>(0x3b, 2, 0, 0x18)); // also fast device
    NkeTopline.addDevice(speed); // also fast device

    // i really need to fix this ptr stuff!
    auto cog = std::make_shared<Nke::Cog>(bridge);
    NkeTopline.addDevice(cog);
    auto sog = std::make_shared<Nke::Sog>(bridge);
    NkeTopline.addDevice(sog);

    // this lambda goes to die..
    // N2KHandlers[128259L] = speed->handleN2kData;/*[speed_ptr = speed.get()](const tN2kMsg &N2kMsg)
    // N2KHandlers.insert({128259L,[speed=speed](const tN2kMsg &N2kMsg) {  speed->handleN2kData(N2kMsg); }});/*
    // N2KHandlers[128259L]=speed;
    // N2KHandlers.emplace(128259L, speed); #################
    /*
    {
      unsigned char SID;
      double SOW;
      double SOG;
      tN2kSpeedWaterReferenceType SWRT;

      if (ParseN2kBoatSpeed(N2kMsg, SID, SOW, SOG, SWRT))
      {
        // if (N2kIsNA(SOW))
        speed_ptr->setSpeed(N2kIsNA(SOW) ? SOW : msToKnots(SOW));
        Serial.printf("Boat Speed %f\n", N2kIsNA(SOW) ? SOW : msToKnots(SOW));
        // OutputStream->print("Boat speed:");
        // TODO deal with SOG later
        // if (SWRT == N2kSWRT_Paddle_wheel ) ???
        // PrintLabelValWithConversionCheckUnDef(" SOW:",N2kIsNA(SOW)?SOW:msToKnots(SOW));
        // PrintLabelValWithConversionCheckUnDef(", SOG:",N2kIsNA(SOG)?SOG:msToKnots(SOG));
        // OutputStream->print(", ");
        // PrintN2kEnumType(SWRT,OutputStream,true);
      }
    };*/

    NkeTopline.addDevice(std::make_shared<Nke::WindAngle>(bridge));

    // Temperature
    // NkeTopline.addDevice(std::make_shared<Nke::Temp>(0x30, 1, 9));

    auto temp = std::make_shared<Nke::WaterTemp>(bridge);

    NkeTopline.addDevice(temp);

    // support multiple pgn's ?
    // todo one pgn to many as well ?
    // N2KHandlers.emplace(130310L, temp); ################
    // No point adding multiple..
    /*N2KHandlers.emplace(130311L,temp);
    N2KHandlers.emplace(130316L,temp);
  */

    // wind speed!

    auto ws = std::make_shared<Nke::WindSpeed>(bridge);
    NkeTopline.addDevice(ws);
    // N2KHandlers.emplace(130306L, ws);
    // ##

    // NkeTopline.addDevice(std::make_shared<Nke::Speed>(0x3b, 2, 4, 0x15));

    // NkeTopline.addDevice()
    /*
      auto genericHandler3b = std::make_shared<GenericHandler>(0x3b, 0x20);
      NkeTopline.addSendData(genericHandler3b->id(), genericHandler3b->data());
      handlers.push_back(genericHandler3b);

      // 0x3c aparent wind angle!
      auto genericHandler3c = std::make_shared<GenericHandler>(0x3c, 0x10);
      NkeTopline.addSendData(genericHandler3c->id(), genericHandler3c->data());
      handlers.push_back(genericHandler3c);

      // announcing on 34 gave bus voltage.. bug ?
      // auto genericHandler34=std::make_shared<GenericHandler>(0x34,0);
      // NkeTopline.addSendData(genericHandler34->id(),genericHandler34->data());
      // handlers.push_back(genericHandler34);

      // 31 is sea temp same as air temp with default offset 0x3e8 (1000)

      // announcing on 34 gave bus voltage.. bug
      auto genericHandler31 = std::make_shared<GenericHandler>(0x31, 0);
      NkeTopline.addSendData(genericHandler31->id(), genericHandler31->data());
      handlers.push_back(genericHandler31);

      // 3a is boat speed.. asume 15 is also boat speed.
      // boat speed has a default offset5 of 1000 (3e8) same with temperature..
      // this went wrong..
      auto genericHandler3a = std::make_shared<GenericHandler>(0x3a, 0);
      NkeTopline.addSendData(genericHandler3a->id(), genericHandler3a->data());
      handlers.push_back(genericHandler3a);
    */
    // auto genericHandler3d=std::make_shared<GenericHandler>(0x3d);

    // NkeTopline.addSendData(genericHandler->id(),genericHandler->data());
    // NkeTopline.addSendData(genericHandler->id(),genericHandler->data());

    // add auto pilot controller!
    // m_auto=std::make_unique<AutopilotController(NkeTopline,bridge); //id is hard coded to 0x2.. maybe we need to work on that
    m_auto = std::unique_ptr<AutopilotController>(new AutopilotController(NkeTopline, bridge));

    NMEA2000.SetOnOpen(OnN2kOpen);
    NkeTopline.Open();

    NMEA2000.Open();

    // run nmea2k on core 0
    xTaskCreatePinnedToCore(N2Kloop, "N2K", 10000, NULL, 1, &TaskN2k, 0);
    xTaskCreatePinnedToCore(ApLoop, "N2K", 10000, NULL, 1, &TaskAp, 1);

    // start listening on serial port 2 for NKE data
    // 9600 is more or les 1 character per millisecond what should the timeout be
    // TODO remember that false/true here causes havoc old / new design old is true new is false
    // Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2,false,2);    //Hardware Serial of ESP32
    // Serial2.setRxTimeout(1);
    // Serial2.onReceive([&Serial2,&Nke](void){

    /*while(Serial2.available())
    {
      auto data=(uint8_t)Serial2.read();
      printf("bytes %02x\n",data);

      //reads++;
    }*/
    // if (reads > 1)

    // size_t available = Serial2.available();
    // if (available > 1)
    // printf("available %d\n",available);
    //
    //    while (available-- > 0) {
    //     auto data=(uint8_t)Serial2.read();
    // ok so this is to slow/deep !! what to do
    // Nke.protocol_decoder(data);
    //});
}

// static int NodeAddress=32;

void sendCommand(uint8_t command, uint8_t dev, uint8_t reg, std::list<uint8_t> data)
{
    tNkeMsg msg;
    int len = 0;
    msg.channel = command;
    msg.data[len++] = dev;
    msg.data[len++] = reg;
    // todo do a max check
    for (auto &a : data)
    {
        msg.data[len++] = a;
    }
    msg.len = len;
    NkeTopline.sendCommand(msg);
}

void loop()
{
    NkeTopline.ParseMessages();
}

// *****************************************************************************
