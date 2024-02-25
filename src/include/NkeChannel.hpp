#pragma once
#include <cstdint>
#include <string>

namespace Nke
{
    enum class Channel : uint8_t
    {
        AWS_FAST = 0x15, // fast aparent wind speed
        // 16 depth ?
        // 17 locked at 180.. no idea what that means
        BS_FAST = 0x18, // boat speed fast
        HEADING_FAST = 0x19,
        DATE_MIN_SEC = 0x1B,
        // 1C FFFF //look at other grabs
        // 1D UNKNOWN //look at other grabs
        RUDDER_FAST = 0x1e, // 0x04xx means other angle
        // 1f speed sensor ?
        // 20 speed sensor ?
        //  28 is 0x00
        DATE_HR_DAY = 0x2F, // first byte hour second day
        HEEL = 0x2b,

        WATER_TEMP = 0x31, // water temperature
        // 34 distance data from log ?
        BS_AVG = 0x3a,      // boat speed avg
        AWS_AVG = 0x3b,     // aparent win speed avg
        AWA = 0x3c,         // SLOW ish DISP AWA
        HEADING_AVG = 0x3d, // Heading slow
        DTW = 0x3E,         // distance to waypoint integer value in nautical miles div 10
        CTW = 0x3F,         // course to waypoint integer value 0-359 Head to WP
        // 40 is 1000 ? meaning what race countdown?
        // value of 40 is written when mode GPS is selected ..
        SOG = 0x41,
        COG = 0x42,
        DATE_YR_MON = 0x45,
        // 46 is 3 now 19 jumps 22 .. no clue!
        // so it changes.. but what is it
        // 47 == heading to WP_SLOW _ .. ok need to look at this one in action

        AP_CONTROL = 0x4E,
        AP_CONFIG = 0x4F,
        // 50 unknown 0xFFFF
        PILOT_POWER = 0x51, // in 0.1 Watts increments 0xFFFF for up to 10 minutes
        // 52 unkown 0xFFFF
        PILOT_STATE = 0x53, // 0x5 idle, 0x3 GPS , 0 compass, 1 aparent wind, 2 rudder
        // 54 unknown 0xFFFF
        PILOT_VOLT = 0x55,
        POS_LAT_HR_MIN = 0x56,
        POS_LAT_SECONDS = 0x57,
        POS_LONG_HR_MIN = 0x58,
        POS_LONG_SECONS = 0x59,
        // shunt sensor
        VOLT = 0x5c,
        AMP = 0x5d,
        CAP_AH = 0x5e,
        CAP_PERCENT = 0x5F,
        TRIM = 0x73,

    };

    // maybe use arduino strings ?
    std::string channelToString(Channel &c);
};