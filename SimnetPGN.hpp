#pragma once

#include <N2kMessages.h>

enum class SimnetApMsgType : uint8_t
{
    // Mode = 0x0a,
    Status = 0x02,
    StatusRequest = 0x03,
    Mode = 0x0a,
    ModeRequest = 0x0b,
    // unsure on these might be oposite but the battern seems to be .. lsb indicating a request!
    // maybe we can encode that in some smart way
    Change = 0x1d,
    // Change = 0x80,
    // ChangeRequest = 0x81,
};
/*
LOOKUP_TYPE_BITFIELD(SIMNET_AP_MODE_BITFIELD, BYTES(2))
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 3, "Standby")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 4, "Heading")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 6, "Nav")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 8, "No Drift")
LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 10, "Wind")
LOOKUP_END
*/
enum class SimnetApMode : uint16_t
{
    // None = 0,
    Standby = 0x1 << 3,
    Heading = 0x1 << 4,
    Nav = 0x1 << 6,
    Nd = 0x1 << 8,
    Wind = 0x1 << 10 // whats aparent and whats true..
};

enum class SimnetApStatus : uint8_t
{
    Manual = 2,
    Automatic = 16,
};

// interesting
/*enum class SimnetApMode : uint16_t
{
    Idle_Heading = 0x0a00,
    Active_Heading = 0x1600,
    Idle_Wind = 0x1e00,
    Active_Wind = 0x0604,
    Idle_Nav = 0x1600,   // folllowed by 0x80 00
    Active_Nav = 0xf000, // followed by 0x80 00
};*/

enum class SimnetApChange : uint8_t
{
    REQ = 0x81,
    REP = 0x80,
};

// can contain different data
struct PGN65305Payload
{
    PGN65305Payload()
    {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
    }
    // this might be all wrong
    PGN65305Payload(const SimnetApStatus &status)
    {
        // uint16_t tmp = static_cast<uint16_t>(status);
        data[0] = static_cast<uint8_t>(status);
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
    }
    PGN65305Payload(const SimnetApMode &mode)
    {
        uint16_t tmp = static_cast<uint16_t>(mode);
        // lsb ahead of msb human error
        data[0] = tmp & 0xFF;
        data[1] = tmp >> 16;
        data[2] = 0;
        data[3] = 0;
        /* if (mode == SimnetApMode::Idle_Nav || mode == SimnetApMode::Active_Nav)
         {
             data[2] = 80;
         }*/
    }
    PGN65305Payload(const SimnetApChange &change)
    {
        data[0] = static_cast<uint8_t>(change);
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
    }

    uint8_t data[4]; // allways a 4 byte payload
};

// enum class Simnet

// enum class SimnetAp

void SetN2kPGN6305(tN2kMsg &msg, const SimnetApMsgType &mode, const PGN65305Payload &payload = PGN65305Payload());

inline void
SetSimnetStatus(tN2kMsg &N2kMsg, const SimnetApStatus &status)
{
    SetN2kPGN6305(N2kMsg, SimnetApMsgType::Status, PGN65305Payload(status));
}

inline void SetSimnetMode(tN2kMsg &N2kMsg, const SimnetApMode &mode)
{
    SetN2kPGN6305(N2kMsg, SimnetApMsgType::Mode, PGN65305Payload(mode) /*PGN65305Payload(status) mode*/);
}

// Might not need this one
inline void SetSimnetStatusChange(tN2kMsg &N2kMsg, const SimnetApChange &change)
{
    SetN2kPGN6305(N2kMsg, SimnetApMsgType::Change, PGN65305Payload(change));
}

enum class SimnetApCommand : uint8_t
{
    Standby = 6,
    Auto = 9, // heading hold
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

    // SetAngle = 28,
    // what is 1c?
    NotifyController = 28, // this is sendt by controllers before the command to address 0xfe
    // should we reply something to this ?

    TimerSync = 61,
    PingPortEnd = 112,
    PingStarboardEnd = 113,

};

enum class SimnetApDirection : uint8_t
{
    Zero = 0,
    Port = 2,
    Starboard = 3,
    LeftRudder = 4,  // port
    RightRudder = 5, // starboard
    None = 0xff,
};

enum class SimnetCommand
{
    Event = 2,
    Alarm = 1,
    Autpilot = 255,
};

/*
TODO deal with the parsing of this object more elegantly
SimnetCommand GetSimnetCommand(tN2kMsg &msg)
{
    if ()
}*/

// parse AP command .. Need to parse AP / Alarm and Event as separate
void ParsePgn130850Ap(const tN2kMsg &msg, uint8_t &addr, SimnetCommand &simnetcmd, SimnetApStatus &apStatus, SimnetApCommand &apcmd, SimnetApDirection &dir, double &angle);
// void ParsePGN130850(tN2kMsg &msg, uint8_t &address, Command &cmd, )

/*enum class Mode63540 : uint8_t
{
  Standby=0x00,
  Heading=0x
};*/

/*
standby => 0x00 or 0x10 in byte 3

*/

enum class SimnetMode
{
    Standby,
    Heading,
    Followup,
    Wind,
    Navigation,

};

void SetPGN65340(tN2kMsg &msg, const SimnetMode &mode /*Mode63540 &mode*/);

inline void printMsg(const tN2kMsg &msg)
{
    Serial.printf("PGN%d,source %d, dst %d, pri %d content: ", msg.PGN, msg.Source, msg.Destination, msg.Priority);
    for (auto i = 0; i < msg.DataLen; i++)
    {
        Serial.printf("%02x,", msg.Data[i]);
    }
    Serial.printf("\n");
}
/*
inline void SetN2kSystemTime(tN2kMsg &N2kMsg, unsigned char SID, uint16_t SystemDate, double SystemTime, tN2kTimeSource TimeSource = N2ktimes_GPS)
{
    SetN2kPGN126992(N2kMsg, SID, SystemDate, SystemTime, TimeSource);
}*/