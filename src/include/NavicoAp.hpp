#pragma once

#include <NMEA2000.h>

#include <deque>
#include "NkeBridge.hpp"

#include "SimnetPGN.hpp"

/*
const unsigned long ReceiveMessages0[] PROGMEM = {126992L // System time
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
*/

// lets start by pretending to be one..

static constexpr unsigned long SIMRAD_AP_PGN = 65305;    // listed
static constexpr unsigned long SIMNET_AP_ANGLE = 65341;  // we dont receive these
static constexpr unsigned long SIMNET_AP_STATUS = 65340; // we dont receive these

// static constexpr unsigned long SIMNET_AP_STATUS2 = 65340;

/*
{{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
{"Reserved", 2, RES_NOTUSED, false, 0, ""},
{"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
{"Autopilot Status 1", BYTES(1), false, "=0x10", "Pilot ON"},
{"Autopilot Mode", BYTES(1), false, LOOKUP_SIMRAD_PILOT_MODE, ""},
{0xfe},
{"Autopilot Status 2", 3, false, "=2", "Pilot ON"},
{"Reserved", 5, RES_NOTUSED, false, 0, "0x1F"},
{"Reserved", BYTES(2), RES_BINARY, false, 0, "0x00, 0x80"}}}*/
// The fuck is this 130845
// 130846
// 65323
// EVENT AP AND EVENT_AP_REPLY! //this is BT-1
// simnetalarm msg..

static constexpr unsigned long SIMNET_AP_COMMAND = 130850;
static constexpr unsigned long SIMNET_AP_REPLY = 130851;

static constexpr unsigned long SIMNET_COMISSION = 130845;

static constexpr unsigned long SIMNET_ALARM_TEXT = 130856;

// same message to and from ?
const unsigned long NavicoApTransmitMessages[] PROGMEM = {
    SIMRAD_AP_PGN,
    SIMNET_AP_STATUS,
    SIMNET_AP_ANGLE,
    SIMNET_AP_REPLY,
    0};

const unsigned long NavicoApRecieveMessages[] PROGMEM = {
    SIMNET_ALARM_TEXT,
    SIMRAD_AP_PGN,
    SIMNET_AP_COMMAND,
    SIMNET_AP_REPLY,
    SIMNET_COMISSION,
    0};

// todo rename to SIMNET_AP!!
// TODO make an abstract N2kAp..
class NavicoAp
{
public:
    NavicoAp(tNMEA2000 &n2k, NkeBridge &bridge, uint32_t uid, int dev_id)
        : m_n2k(n2k), m_bridge(bridge), m_scheduler(false, 1000, 100)
          // every 25ms see if we should send a fast packet ...
          ,
          m_fast_scheduler(false, 25, 10), m_uid(uid), m_dev_id(dev_id)
    {
        n2k.SetProductInformation("00000002",              // Manufacturer's Model serial code
                                  100,                     // maybe 100 is the right here?
                                  "FakeNavicoAp",          // Manufacturer's Model ID
                                  "1.2.0.24 (2022-10-01)", // Manufacturer's Software version code
                                  "1.2.0.0 (2022-10-01)",  // Manufacturer's Model version
                                  0xff,                    // load equivalency - use default
                                  0xffff,                  // NMEA 2000 version - use default
                                  0xff,                    // Sertification level - use default
                                  m_dev_id                 /// dev id
        );
        // probably not so important.. maybe
        //  Set device information https://manualzz.com/doc/12647142/nmea2000-class-and-function-codes
        n2k.SetDeviceInformation(uid, // Unique number. Use e.g. Serial number.
                                 150, // Autopilot has func 150 //130, // 150, // Device function Bridge
                                 40,  // steering and control surfaces //25,  // intranetwork device

                                 // 130, // Device function=Atmospheric. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                 // 85, // Device class=External Environment. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                 1857, // has to be simrad ////2046,     this is manufacturer code
                                 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                 4,       // Marine
                                 m_dev_id // dev id
        );

        n2k.ExtendTransmitMessages(NavicoApTransmitMessages, m_dev_id);
        n2k.ExtendReceiveMessages(NavicoApRecieveMessages, m_dev_id);
    }
    /*
        enum class Command : uint8_t
        {
            Standby = 6,
            Auto = 9,
            Nav = 10,
            NonFollowUp = 13,
            FollowUp = 14,
            Wind = 15,
            SquareTurn = 18,
            CTurn = 19,
            UTurn = 20,
            SpiralTurn = 21,
            ZigZagTurn = 22,
            LazySTurn = 23,
            DepthTurn = 24,
            CourseChange = 26,
            SetAngle = 28,
            TimerSync = 61,
            PingPortEnd = 112,
            PingStarboardEnd = 113,

        };
    */
    void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
    {
        if (N2kMsg.PGN == SIMNET_ALARM_TEXT)
        {
            int index = 0;
            Serial.printf("Alarm Text received length %d\n", N2kMsg.DataLen);
            uint16_t manufacturer = N2kMsg.Get2ByteUInt(index);
            uint16_t messageId = N2kMsg.Get2ByteUInt(index);
            uint8_t unknown = N2kMsg.GetByte(index);
            uint8_t unknow2 = N2kMsg.GetByte(index);
            // TODO convert to std::string view then print as c_str
            Serial.printf("%s\n", &N2kMsg.Data[6]);
        }

        if (N2kMsg.PGN == SIMNET_COMISSION)
        {
          int index=0;
          uint16_t top = N2kMsg.Get2ByteUInt(index);
          printf("Manufacturer %d\n", top & 0x7FF);
          printf("Device Type %d\n", top >> 13 & 0x7);

          unsigned char Target = N2kMsg.GetByte(index);
          printf("Target %d\n",Target);

          Serial.printf("Received simnet commission from %d with len %d\n", N2kMsg.Source, N2kMsg.DataLen);
        }
        // switch(PGNS) to handle
        if (N2kMsg.PGN == SIMRAD_AP_PGN)
        {
            Serial.printf("Received PGN %u from %d\n", SIMRAD_AP_PGN, N2kMsg.Source);
            int index = 0;
            uint16_t mfc = N2kMsg.Get2ByteUInt(index);
            uint8_t model = N2kMsg.GetByte(index);
            uint8_t mode = N2kMsg.GetByte(index);

            switch (static_cast<SimnetApMsgType>(mode))
            {
            case SimnetApMsgType::ModeRequest:
                sendMode();
                break;
            case SimnetApMsgType::StatusRequest:
                sendStatus();
                break;
            default:
                Serial.printf("unexpected msg %02\n", mode);
                break;
            }

            // analyze it
        }
        if (N2kMsg.PGN == SIMNET_AP_COMMAND)
        {
            uint8_t status;
            uint8_t command;
            uint8_t dummy;
            uint8_t direction;
            double angle;

            tN2kMsg reply;

            if (N2kMsg.Data[2] == 0xfe)
            {
                // forward broadcasted command to dest
                reply.SetPGN(SIMNET_AP_COMMAND);
                for (auto i = 0; i < N2kMsg.DataLen; i++)
                {
                    if (i == 2)
                    {
                        reply.Data[i] = m_n2k.GetN2kSource(m_dev_id);
                    }
                    else
                    {
                        reply.Data[i] = N2kMsg.Data[i];
                    }
                }
            }
            else
            {
                reply.SetPGN(SIMNET_AP_REPLY); // maybe set pgn clears ?
                for (auto i = 0; i < N2kMsg.DataLen; i++)
                {
                    reply.Data[i] = N2kMsg.Data[i];
                }
            }
            reply.Priority = 7;
            reply.DataLen = N2kMsg.DataLen;

            // memcpy(reply.Data, N2kMsg.Data, N2kMsg.DataLen);

            /*
            if (reply.Data[2] == 0xfe)
            {
                reply.Destination = N2kMsg.Source;
            }*/

            // lets try replying something fun by setting the source id before replying that didnt work
            /* if (reply.Data[2] == 0xfe)
             {
                 // asume Fe 1c is a forward to a controller to send control signals.!

                 reply.Data[2] = 35;
                 reply.PGN = SIMNET_AP_COMMAND;
                 // reply.SetPGN(SIMNET_AP_COMMAND);
                 m_replies.push_back(SIMNET_AP_COMMAND);
                 // N2kMsg.Data[2] = 35;
                 reply.SetPGN(SIMNET_AP_REPLY); // maybe set pgn clears ?

                 //    sendN2kMsg(reply);
                 // No idea how to deal with this!
                 //  reply.Data[2] = N2kMsg.Source;
                 // reply.Data[2] = 35; // our id
             }*/

            // add our id in the reply this was the missing magic! we can probably store this in a variable but who cares
            // reply.addByte(m_n2k.GetN2kSource());
            // id 3 of reply should be the AP controller however this is also the AP command according to canboat!!
            // might be totally wrong i have no idea!
            // i think the main issue is that the ap sends to 254 and not 35..
            // orca shows state as 254.. might be relateed
            // reply.Data[2] = 35;
            // schedule a msg
            // printMsg(reply);

            m_replies.push_back(reply);
            /*Serial.printf("Received AP_COMMAND %u\n", SIMNET_AP_COMMAND);
            Serial.printf("Source %d\n", N2kMsg.Source);    // one matches controller
            Serial.printf("Dest %d\n", N2kMsg.Destination); // 255 meaning its a broadcast
            Serial.printf("Bytes %d\n", N2kMsg.DataLen);    // 12 matches expected
*/
            // Serial.printf("Bytes %d\n", N2kMsg.DataLen);
            //  lets asume this is 11 bytes.. long.. no idea if that asumption is correct..
            printMsg(N2kMsg);

            if (N2kMsg.DataLen == 12) // why is len 12 for 11 bytes ?
            {
                uint8_t address;
                SimnetCommand cmd;
                SimnetApDirection direction;
                SimnetApMode mode;
                SimnetApStatus status;
                SimnetApCommand apcmd;
                double angle;
                ParsePgn130850Ap(N2kMsg, address, cmd, status, apcmd, direction, angle);

                switch (apcmd)
                {
                case SimnetApCommand::Standby: // 6 set standby
                    setEngaged(false);
                    break;
                case SimnetApCommand::Auto: // TODO rename to HeadingHold or just Heading
                    Serial.printf("Set Mode Heading\n");
                    m_angle = m_bridge.heading().N2k();
                    m_mode = ApMode::Heading;
                    setEngaged(true);
                    break;
                case SimnetApCommand::Wind:
                    m_angle = m_bridge.windAngle().N2k();
                    m_mode = ApMode::Wind;
                    setEngaged(true);
                    break;
                case SimnetApCommand::CourseChange:
                    switch (direction)
                    {
                        // maybe we should use integer radian precision instead of double!
                    case SimnetApDirection::Port:
                        //+10 d106 (06d1) which is 1745 with precision 0.0001 thats 0.1745 which is 10 degrees in radians
                        //+1 ae00 (00ae) = 174 with precision 0.0001 thats 0.0174 which is 1 degree in radians
                        m_angle -= angle;
                        break;
                    case SimnetApDirection::Starboard:
                        m_angle += angle;
                        break;
                    }

                    printf("NewAgle %f\n", (180 * m_angle) / M_PI);

                    break;
                    // what is this issued both on standby and on auto before .. standby and auto msg
                case SimnetApCommand::NotifyController: // Dont thing this is the command
                    // maybe we should reply the angle?
                    // printMsg(N2kMsg);

                    if (angle == 0.000000)
                    {
                        switch (m_mode)
                        {
                        case ApMode::Wind:
                            m_angle = m_bridge.windAngle().N2k();
                            break;
                        case ApMode::Heading:
                            m_angle = m_bridge.heading().N2k();
                            break;
                        }

                        // fetch angle for mode or maybe its something else entirely .. ?
                    }
                    printf("SetAngle %f\n", (180 * m_angle) / M_PI);
                    break;
                default:
                    Serial.printf("Unhandled SimnetApCommand %02x(%d)\n", static_cast<uint8_t>(apcmd), static_cast<uint8_t>(apcmd));
                    break;
                    //
                }
                /*int index = 0;
                uint16_t top = N2kMsg.Get2ByteUInt(index);
                Serial.printf("Source %d\n", N2kMsg.Source);    // one matches controller
                Serial.printf("Dest %d\n", N2kMsg.Destination); // 255 meaning its a broadcast

                printf("Manufacturer %d\n", top & 0x7FF);
                printf("Device Type %d\n", top >> 13 & 0x7);
                uint8_t address = N2kMsg.GetByte(index);
                Serial.printf("reserved %02x\n", N2kMsg.GetByte(index)); // reserved
                uint8_t command_id = N2kMsg.GetByte(index);
                // need to figure out how many commands to support
                Serial.printf("address id %d\n", address);

                Serial.printf("Command id %d\n", command_id);

                if (command_id == 255)
                {
                    Serial.printf("Ap status %02x\n", N2kMsg.GetByte(index));
                    uint8_t command = N2kMsg.GetByte(index);
                    Serial.printf("Ap command %02x\n", command);
                    N2kMsg.GetByte(index); // spare
                    uint8_t direction = N2kMsg.GetByte(index);
                    Serial.printf("Ap direction %02x\n", direction);
                    double angle = N2kMsg.Get2ByteUDouble(0.0001, index);
                    Serial.printf("Ap angle %f\n", angle);

                    switch (static_cast<Command>(command))
                    {
                    case Command::Standby:
                        m_mode = Mode::Standby;
                        break;
                    case Command::Auto:
                        m_mode = Mode::Heading;
                        // get target angle
                        m_angle = m_bridge.heading().N2k();
                        break;
                    case Command::Wind:
                        m_mode = Mode::Wind;
                        // get target angle
                        m_angle = m_bridge.windAngle().N2k();
                        break;
                    case Command::CourseChange:

                        // make sure we are in the 0-2PI range
                        m_angle = std::fmod((m_angle + 2 * M_PI), 2 * M_PI);

                        break;
                    case Command::SetAngle:
                        m_angle = angle;
                        break;
                    }
                    switch (static_cast<Direction>(direction))
                    {
                        // 00
                    case Direction::Zero:

                        // fetch value from N2kBridge ?
                        break;
                    case Direction::Port:
                        // set course!
                        // need to modulus 2PI probably
                        m_angle -= angle;
                        break;
                    case Direction::Starboard:
                        m_angle += angle;
                        break;
                    case Direction::None:
                        break;
                    default:
                        printf("unknown direction %02x", direction);
                        break;
                        //  case
                    }
                }*/
                // maybe queue for both commands before sending ?
                /*m_n2k.SetDebugMode(tNMEA2000::tDebugMode::dm_ClearText);
                m_n2k.SendMsg(reply, m_dev_id);
                m_n2k.SetDebugMode(tNMEA2000::tDebugMode::dm_None);
*/
                // m_replies.push_back(reply);
            }

            if (N2kMsg.DataLen == 11)
            {
                int index = 0;
                uint16_t top = N2kMsg.Get2ByteUInt(index);
                /*uint8_t reserved = */ N2kMsg.GetByte(index);
                uint8_t command_id = N2kMsg.GetByte(index);
                // need to figure out how many commands to support
                Serial.printf("Command id %d\n", command_id);
                // Serial.printf
            }
        }
    }
    void setEngaged(bool engaged)
    {
        if (m_engaged != engaged)
        {
            Serial.printf("Send StateChange\n");
            // send 81 then set on internally
            // send 80 once on internally
            tN2kMsg EightOne;
            SetSimnetStatusChange(EightOne, SimnetApChange::REQ);
            tN2kMsg EightZero;
            SetSimnetStatusChange(EightZero, SimnetApChange::REP);
            m_replies.push_back(EightOne);
            m_replies.push_back(EightZero);
            m_engaged = engaged;
        }
    }

    /* LOOKUP_TYPE_BITFIELD(SIMNET_AP_MODE_BITFIELD, BYTES(2))
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 3, "Standby")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 4, "Heading")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 6, "Nav")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 8, "No Drift")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 10, "Wind")
     LOOKUP_END*/
    /*
        enum class SimnetStatus : uint16_t
        {
            Standby = 0x1 << 3,
            Heading = 0x1 << 4,
            Nav = 0x1 << 6,
            Nd = 0x1 << 8,
            Wind = 0x1 << 10 // whats aparent and whats true..
        };
    */
    // same as above but encoded as 8 bit
    // i am guessing there are more.. like Forced rudder TrueWind.. etc
    enum class ApMode : uint8_t
    {
        Heading = 2,
        Wind = 3,
        Nav = 10,
        NoDrift = 11
    };

    // LOOKUP(SIMNET_DEVICE_REPORT, 10, "Mode")
    // LOOKUP(SIMNET_DEVICE_REPORT, 11, "Send Mode")
    //  start the ap controller ?
    void start()
    {
        m_scheduler.UpdateNextTime();
        m_fast_scheduler.UpdateNextTime();
        // start timers
    }

    // todo run with task executor ?
    // TODO fix timers with callbacks and periodic this design is bad
    void process()
    {
        if (m_scheduler.IsTime())
        {
            // sendStatus();
            // sendMode();
            sendAngle();
            sendMore();
            // sendReply();
            m_scheduler.UpdateNextTime();
        }

        if (m_fast_scheduler.IsTime())
        {
            // if a reply is queued send it
            if (!m_replies.empty())
            {
                // Serial.printf("SendingReply\n");
                auto msg = m_replies.front();

                msg.Source = m_n2k.GetN2kSource(m_dev_id);
                //; // todo get source addr for dev %d
                sendN2kMsg(msg);
                //    m_n2k.SendMsg(m_replies.front(), m_dev_id);
                m_replies.pop_front();
            }
            m_fast_scheduler.UpdateNextTime();
        }
    }

private:
    tNMEA2000 &m_n2k;
    NkeBridge &m_bridge;

    void sendN2kMsg(tN2kMsg &msg)
    {
        //printMsg(msg);

        m_n2k.SendMsg(msg, m_dev_id);
    }

    void sendMore()
    {
        // AC12_PGN130850
        /*
        function AC12_PGN130860 () {
  const message = "%s,7,130860,%s,255,21,13,99,ff,ff,ff,ff,7f,ff,ff,ff,7f,ff,ff,ff,ff,ff,ff,ff,7f,ff,ff,ff,7f"
  msg = util.format(message, (new Date()).toISOString(), canbus.candevice.address)
  canbus.sendPGN(msg)
}
        */
    }

    void sendStatus()
    {
        // TODO make a class parser for these PGN's
        //"%s,7,65305,%s,255,8,
        //  41,9f
        //,00,0a,0a,00,00,00" ]

        // send our status
        tN2kMsg msg;
        if (m_engaged)
        {
            SetSimnetStatus(msg, SimnetApStatus::Automatic);
            // SimnetApStatus::Automatic;
        }
        else
        {
            SetSimnetStatus(msg, SimnetApStatus::Manual);
        }
        m_replies.push_back(msg);
        // m_n2k.SendMsg(msg, m_dev_id);

        // wait 25 milli before sending next status ?
    }

    void sendMode()
    {
        tN2kMsg msg;

        // setSimnetMode(msg, m_mode);
        //  GOD this is ugly
        //  if (m_engaged)
        //{
        if (m_engaged)
        {
            switch (m_mode)
            {
            case ApMode::Heading:
                SetSimnetMode(msg, SimnetApMode::Heading);
                break;
            case ApMode::Wind:
                SetSimnetMode(msg, SimnetApMode::Wind);
                break;
            case ApMode::Nav:
                SetSimnetMode(msg, SimnetApMode::Nav);
                break;
            default:
                Serial.printf("Unknown ApMode %d\n", m_mode);
            }
        }
        else
        {
            SetSimnetMode(msg, SimnetApMode::Standby);
        }
        /*}
        else
        {
            switch (m_mode)
            {
            case ApMode::Heading:
                SetSimnetMode(msg, SimnetApMode::Idle_Heading);
                break;
            case ApMode::Wind:
                SetSimnetMode(msg, SimnetApMode::Idle_Wind);
                break;
            case ApMode::Nav:
                SetSimnetMode(msg, SimnetApMode::Idle_Nav);
                break;
            }
        }*/
        /*  if (msg.DataLen > 0)
          {
              printMsg(msg);
          }*/

        m_replies.push_back(msg);

        // sendN2kMsg(msg);
    }

    void sendAngle()
    {
        tN2kMsg msg1;
        // if we emulate AC42 we should send PGN65340
        if (m_engaged)
        {
            switch (m_mode)
            {
            case ApMode::Heading:
                SetPGN65340(msg1, SimnetMode::Heading);
                break;
            case ApMode::Wind:
                SetPGN65340(msg1, SimnetMode::Wind);
                break;
            case ApMode::Nav:
                SetPGN65340(msg1, SimnetMode::Navigation);
                break;

            default:
                Serial.printf("Not sending PGN65340 \n");
            }
        }
        else
        {
            SetPGN65340(msg1, SimnetMode::Standby);
        }
        // printMsg(msg1);

        m_replies.push_back(msg1);

        // SIMNET_AP_STATUS
        //   tN2kMsg msg1;
        //    6 byte status msg
        //    msg1.SetPGN(SIMNET_AP_STATUS);
        //    msg1.Priority = 3;
        //    msg.Add2ByteUInt(1857);
        //    msg1.AddByte(0xff);                          // broadcast
        //    msg1.AddByte(34);                            // length or PGN address ?
        //    msg1.Add2ByteUInt(1857 | (0x4 << (11 + 2))); // manufacturer
        /*if (m_mode == Mode::Standby)
        {
            msg1.AddByte(0x00); // length or AP id ?
        }
        else
        {
            msg1.AddByte(0x10); // length or AP id ? //0A for 65302
        }
        switch (m_mode)
        {
        case Mode::Standby:
            msg1.AddByte(0x00);
            msg1.AddByte(0xFE);
            msg1.AddByte(0xF8);
            break;
        case Mode::Heading:
            msg1.AddByte(0x1);
            msg1.AddByte(0xFE);
            msg1.AddByte(0xFA);
            break;
        case Mode::Wind:
            msg1.AddByte(0x3);
            msg1.AddByte(0xFE);
            msg1.AddByte(0xFA);
            break;

        case Mode::Nav:
            msg1.AddByte(0x06);
            msg1.AddByte(0xFE);
            msg1.AddByte(0xFA);
            break;
        }

        msg1.AddByte(0x00);
        msg1.AddByte(0x80);

        m_n2k.SendMsg(msg1, m_dev_id);*/
        // only send when active

        // Reports the heading or apparent wind angle dependent on the autopilot mode
        // mode 03 02 and 0b         NoDrift = 11
        if (m_engaged)
        {
            tN2kMsg msg;

            switch (m_mode)
            {
            case ApMode::Wind:
            case ApMode::Heading:
            case ApMode::NoDrift:
            {
                // 6 byte status msg
                msg.SetPGN(SIMNET_AP_ANGLE);
                // msg.SetPGN(65431UL);
                msg.Priority = 6;
                // msg.Add2ByteUInt(1857);
                msg.Add2ByteUInt(1857 | 0x8000 /*Maritime id*/ | 0x1800 /*reserved*/);

                // uint16_t simnet_id = 1857;
                // msg.Add2ByteUInt(simnet_id | 0x8000);

                msg.Add2ByteUInt(0xFFFF); // reserved

                msg.AddByte(static_cast<uint8_t>(m_mode)); // AP_MODE so probably need to store that)
                msg.AddByte(0xFF);                         // reserved
                msg.Add2ByteDouble(m_angle, 0.0001);
                // Serial.printf("Sending Angle %f\n", m_angle);
                /*
                        msg.AddByte(0);                                // 0 for AC and 1 for something                             // model AC 100 for NAC 1 for other.. probably all work
                        msg.AddByte(static_cast<uint8_t>(TYPE::Mode)); //
                        msg.Add2ByteUInt(static_cast<uint16_t>(m_mode));

                        // msg.AddByte(static_cast<uint16_t>(m_mode));
                        //  msg.Add2ByteDouble(bridge.rudderAngle().N2k(), 0.0001);
                        //  msg.Add2ByteDouble(bridge.rudderAngle().N2k(), 0.0001);
                        msg.AddByte(0xff); // Reserved
                        msg.AddByte(0xff); // Reserved
                        // msg.AddByte(0xff); // Reserved
                        // msg.AddByte(0xff); // Reserved
                */
                // sendN2kMsg(msg);
                m_replies.push_back(msg);
            }
            break;
            default:
                break;
            }
        }

        //  m_n2k.SendMsg(msg, m_dev_id);
    }
    /*
        void sendReply()
        {
            while (!m_replies.empty())
            {
                // auto rep = m_replies.front();
                Serial.printf("SendingReply\n");
                m_n2k.SendMsg(m_replies.front(), m_dev_id);
                m_replies.pop_front();
            }
        }
    */
    tN2kSyncScheduler m_scheduler;
    tN2kSyncScheduler m_fast_scheduler;

    uint32_t m_uid;
    int m_dev_id;
    // software mode state!
    ApMode m_mode = ApMode::Heading;

    // Status m_status = Status::Automatic;
    //  SimnetStatus m_mode = Mode::Standby;
    // SimnetStatus m_apState = SimnetStatus::Heading;
    // ApMode m_ApMode = ApMode::Heading; // default is heading
    // think we neeed to report angle based on mode.. so either heading or wind..

    std::deque<tN2kMsg> m_replies;

    double m_angle = 0.00;

    bool m_engaged = false;
};

/*

 {"Simnet: AP Unknown 1",
     65302,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {COMPANY(1857),
      UINT8_FIELD("A"),
      UINT8_FIELD("B"),
      UINT16_FIELD("C"),
      UINT8_FIELD("D"),
      RESERVED_FIELD(BYTES(1)),
      END_OF_FIELDS},
     .interval    = 1000,
     .explanation = "Seen as sent by AC-42 only so far."}

    ,
    {"Simnet: Device Status",
     65305,
     PACKET_LOOKUPS_UNKNOWN,
     PACKET_SINGLE,
     {COMPANY(1857),
      LOOKUP_FIELD("Model", BYTES(1), SIMNET_DEVICE_MODEL),
      MATCH_LOOKUP_FIELD("Report", BYTES(1), 2, SIMNET_DEVICE_REPORT),
      LOOKUP_FIELD("Status", BYTES(1), SIMNET_AP_STATUS),
      SPARE_FIELD(BYTES(3)),
      END_OF_FIELDS},
     .interval    = 1000,
     .explanation = "This PGN is reported by an Autopilot Computer (AC/NAC)"}

    ,
    {"Simnet: Device Status Request",
     65305,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {COMPANY(1857),
      LOOKUP_FIELD("Model", BYTES(1), SIMNET_DEVICE_MODEL),
      MATCH_LOOKUP_FIELD("Report", BYTES(1), 3, SIMNET_DEVICE_REPORT),
      SPARE_FIELD(BYTES(4)),
      END_OF_FIELDS},
     .interval    = 1000,
     .explanation = "This PGN is sent by an active AutoPilot head controller (AP, MFD, Triton2)."
                    " It is used by the AC (AutoPilot Controller) to verify that there is an active controller."
                    " If this PGN is not sent regularly the AC may report an error and go to standby."}

    ,
    {"Simnet: Pilot Mode",
     65305,
     PACKET_LOOKUPS_UNKNOWN,
     PACKET_SINGLE,
     {COMPANY(1857),
      LOOKUP_FIELD("Model", BYTES(1), SIMNET_DEVICE_MODEL),
      MATCH_LOOKUP_FIELD("Report", BYTES(1), 10, SIMNET_DEVICE_REPORT),
      BITLOOKUP_FIELD("Mode", BYTES(2), SIMNET_AP_MODE_BITFIELD),
      SPARE_FIELD(BYTES(2)),
      END_OF_FIELDS},
     .interval    = 1000,
     .explanation = "This PGN is reported by an Autopilot Computer (AC/NAC)"}

    ,
    {"Simnet: Device Mode Request",
     65305,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     {COMPANY(1857),
      LOOKUP_FIELD("Model", BYTES(1), SIMNET_DEVICE_MODEL),
      MATCH_LOOKUP_FIELD("Report", BYTES(1), 11, SIMNET_DEVICE_REPORT),
      SPARE_FIELD(BYTES(4)),
      END_OF_FIELDS},
     .interval    = 1000,
     .explanation = "This PGN is sent by an active AutoPilot head controller (AP, MFD, Triton2)."
                    " It is used by the AC (AutoPilot Controller) to verify that there is an active controller."
                    " If this PGN is not sent regularly the AC may report an error and go to standby."}

    ,
    {"Simnet: Sailing Processor Status",
     65305,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     {COMPANY(1857),
      LOOKUP_FIELD("Model", BYTES(1), SIMNET_DEVICE_MODEL),
      MATCH_LOOKUP_FIELD("Report", BYTES(1), 23, SIMNET_DEVICE_REPORT),
      BINARY_FIELD("Data", BYTES(4), ""),
      END_OF_FIELDS},
     .interval    = 1000,
     .explanation = "This PGN has been seen to be reported by a Sailing Processor."}

LOOKUP_TYPE(SIMNET_AP_STATUS, BYTES(1))
LOOKUP(SIMNET_AP_STATUS, 2, "Manual")
LOOKUP(SIMNET_AP_STATUS, 16, "Automatic")
LOOKUP_END


LOOKUP_TYPE(SIMNET_AP_MODE, BYTES(1))
LOOKUP(SIMNET_AP_MODE, 2, "Heading")
LOOKUP(SIMNET_AP_MODE, 3, "Wind")
LOOKUP(SIMNET_AP_MODE, 10, "Nav")
LOOKUP(SIMNET_AP_MODE, 11, "No Drift")
LOOKUP_END

LOOKUP_TYPE_BITFIELD(SIMNET_AP_MODE_BITFIELD, BYTES(2))
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 3, "Standby")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 4, "Heading")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 6, "Nav")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 8, "No Drift")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 10, "Wind")
LOOKUP_END

LOOKUP_TYPE(SIMNET_DEVICE_MODEL, BYTES(1))
LOOKUP(SIMNET_DEVICE_MODEL, 0, "AC")
LOOKUP(SIMNET_DEVICE_MODEL, 1, "Other device")
LOOKUP(SIMNET_DEVICE_MODEL, 100, "NAC")
LOOKUP_END

LOOKUP_TYPE(SIMNET_DEVICE_REPORT, BYTES(1))
LOOKUP(SIMNET_DEVICE_REPORT, 2, "Status")
LOOKUP(SIMNET_DEVICE_REPORT, 3, "Send Status")
LOOKUP(SIMNET_DEVICE_REPORT, 10, "Mode")
LOOKUP(SIMNET_DEVICE_REPORT, 11, "Send Mode")
LOOKUP(SIMNET_DEVICE_REPORT, 23, "Sailing Processor Status")
LOOKUP_END

LOOKUP_TYPE(SIMNET_AP_EVENTS, BYTES(1))
LOOKUP(SIMNET_AP_EVENTS, 6, "Standby")
LOOKUP(SIMNET_AP_EVENTS, 9, "Auto mode")
LOOKUP(SIMNET_AP_EVENTS, 10, "Nav mode")
LOOKUP(SIMNET_AP_EVENTS, 13, "Non Follow Up mode")
LOOKUP(SIMNET_AP_EVENTS, 14, "Follow Up mode")
LOOKUP(SIMNET_AP_EVENTS, 15, "Wind mode")
LOOKUP(SIMNET_AP_EVENTS, 18, "Square (Turn)")
LOOKUP(SIMNET_AP_EVENTS, 19, "C-Turn")
LOOKUP(SIMNET_AP_EVENTS, 20, "U-Turn")
LOOKUP(SIMNET_AP_EVENTS, 21, "Spiral (Turn)")
LOOKUP(SIMNET_AP_EVENTS, 22, "Zig Zag (Turn)")
LOOKUP(SIMNET_AP_EVENTS, 23, "Lazy-S (Turn)")
LOOKUP(SIMNET_AP_EVENTS, 24, "Depth (Turn)")
LOOKUP(SIMNET_AP_EVENTS, 26, "Change course")
LOOKUP(SIMNET_AP_EVENTS, 61, "Timer sync")
LOOKUP(SIMNET_AP_EVENTS, 112, "Ping port end")
LOOKUP(SIMNET_AP_EVENTS, 113, "Ping starboard end")
LOOKUP_END


LOOKUP_TYPE(SIMNET_EVENT_COMMAND, BYTES(1))
LOOKUP(SIMNET_EVENT_COMMAND, 1, "Alarm")
LOOKUP(SIMNET_EVENT_COMMAND, 2, "AP command")
LOOKUP(SIMNET_EVENT_COMMAND, 255, "Autopilot")
LOOKUP_END

*/

/* Dont think we need these
// There are other bits in use, but their meaning is not yet clear.
LOOKUP_TYPE_BITFIELD(SIMNET_ALERT_BITFIELD, BITS(64))
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 0, "No GPS fix")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 2, "No active autopilot control unit")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 4, "No autopilot computer")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 6, "AP clutch overload")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 8, "AP clutch disengaged")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 10, "Rudder controller fault")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 12, "No rudder response")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 14, "Rudder drive overload")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 16, "High drive supply")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 18, "Low drive supply")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 20, "Memory fail")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 22, "AP position data missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 24, "AP speed data missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 26, "AP depth data missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 28, "AP heading data missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 30, "AP nav data missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 32, "AP rudder data missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 34, "AP wind data missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 36, "AP off course")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 38, "High drive temperature")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 40, "Drive inhibit")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 42, "Rudder limit")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 44, "Drive computer missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 46, "Drive ready missing")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 48, "EVC com error")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 50, "EVC override")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 52, "Low CAN bus voltage")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 54, "CAN bus supply overload")
LOOKUP_BITFIELD(SIMNET_ALERT_BITFIELD, 56, "Wind sensor battery low")
LOOKUP_END



Hei, simulering av AP info:

Du må broadcaste:

PGN 65341
PGN 65305

Core broadcaster for å sette mode og angle:

PGN 130850

Fra vår embedded-utvikler:

Those are proprietary PGNs, Simrad uses manufacturer code 1857 (and 275 in some cases).
You need to set that in the broadcast for Core to recognize the AP as Simrad AP.

Nav mode uses standard PGNs:

XTE: 129283
Navigation Data: 129284
WP Information: 129285

*/

/*
19:24:37.612 -> Received simnet commission from 12 with len 14
19:24:37.612 -> Received simnet commission from 11 with len 14
19:24:38.330 -> PGN65305,source 35, dst 255, pri 7 content: 41,9f,00,02,02,00,00,00,
19:24:38.361 -> PGN65305,source 35, dst 255, pri 7 content: 41,9f,00,0a,08,00,00,00,
19:24:38.361 -> PGN65340,source 35, dst 255, pri 3 content: 41,9f,00,00,fe,f8,00,80,
19:24:38.800 -> PGN130850,source 12, dst 255, pri 2 content: 41,9f,ff,ff,01,17,4a,00,ff,ff,ff,ff,
19:24:38.800 -> Unhandled SimnetApCommand 4a(74)
19:24:38.800 -> PGN130851,source 35, dst 255, pri 7 content: 41,9f,ff,ff,01,17,4a,00,ff,ff,ff,ff,
19:24:38.838 -> Received simnet commission from 12 with len 14
19:24:38.838 -> Received simnet commission from 11 with len 14

So we do receive a initial AP request for 01,17 ..
my guess is that 01 is controller adddress in this case and 17 is unknown

*/