#pragma once

#include <NMEA2000.h>

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

static constexpr unsigned long SIMRAD_AP_PGN = 65305;
static constexpr unsigned long SIMNET_AP_COMMAND = 130850;

// same message to and from ?
const unsigned long NavicoApTransmitMessages[] PROGMEM = {SIMRAD_AP_PGN, 0};

const unsigned long NavicoApRecieveMessages[] PROGMEM = {SIMRAD_AP_PGN, SIMNET_AP_COMMAND, 0};

class NavicoAp
{
public:
    NavicoAp(tNMEA2000 &n2k, uint32_t uid, int dev_id)
        : m_n2k(n2k), m_scheduler(false, 1000, 100), m_uid(uid), m_dev_id(dev_id)
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
    void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
    {
        // switch(PGNS) to handle
        if (N2kMsg.PGN == SIMRAD_AP_PGN)
        {
            Serial.printf("Received PGN %u\n", SIMRAD_AP_PGN);
            // analyze it
        }
        if (N2kMsg.PGN == SIMNET_AP_COMMAND)
        {
            uint8_t status;
            uint8_t command;
            uint8_t dummy;
            uint8_t direction;
            double angle;

            Serial.printf("Received AP_COMMAND %u\n", SIMNET_AP_COMMAND);
            Serial.printf("Source %d\n", N2kMsg.Source);    // one matches controller
            Serial.printf("Dest %d\n", N2kMsg.Destination); // 255 meaning its a broadcast
            Serial.printf("Bytes %d\n", N2kMsg.DataLen);    // 12 matches expected

            int index = 0;

            /*status = N2kMsg.GetByte(index);
            command = N2kMsg.GetByte(index);
            dummy = N2kMsg.GetByte(index);
            direction = N2kMsg.GetByte(index);
            angle = N2kMsg.Get2ByteUDouble(0.0001, index);
*/
            // Serial.printf("status %02x, command %02x,dummy %02x, direction %02x, angle %f\n", status, command, dummy, direction, angle);
            for (int i = 0; i < N2kMsg.DataLen; i++)
            {
                uint8_t data = N2kMsg.GetByte(index);
                printf("Byte %d val %02x %d\n", i, data, data);

                // 41 9f swapped is 9f41 remove the top 6 bits and we get 1857 which is simrad for the first bytes
                // unsure what endian ordering that is
            }
            // AP status..
            /*
            SIMNET_AP_STATUS (0 - 255)
Value	Description
2	Manual
16	Automatic
*/
            // AP command
            /*
            SIMNET_AP_EVENTS (0 - 255)
            Value	Description
            6	Standby
            9	Auto mode
            10	Nav mode
            13	Non Follow Up mode
            14	Follow Up mode
            15	Wind mode
            18	Square (Turn)
            19	C-Turn
            20	U-Turn
            21	Spiral (Turn)
            22	Zig Zag (Turn)
            23	Lazy-S (Turn)
            24	Depth (Turn)
            26	Change course
            61	Timer sync
            112	Ping port end
            113	Ping starboard end

            */

            // reserved
            // direction
            /*
            2	Port
3	Starboard
4	Left rudder (port)
5	Right rudder (starboard)

            */
            // angle 16 bit rad double.
            // angle = msg.Get2ByteUDouble(0.0001, Index);
        }
    }
    enum class TYPE : uint8_t
    {
        Status = 2,
        StatusRequest = 3,
        Mode = 10,
        ModeRequest = 11,
    };

    enum class Status : uint8_t
    {
        Manual = 2,
        Automatic = 16,
    };

    /* LOOKUP_TYPE_BITFIELD(SIMNET_AP_MODE_BITFIELD, BYTES(2))
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 3, "Standby")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 4, "Heading")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 6, "Nav")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 8, "No Drift")
     LOOKUP_BITFIELD(SIMNET_AP_MODE_BITFIELD, 10, "Wind")
     LOOKUP_END*/

    enum class Mode : uint16_t
    {
        Standby = 0x1 << 3,
        Heading = 0x1 << 4,
        Nav = 0x1 << 6,
        Nd = 0x1 << 8,
        Wind = 0x1 << 10 // whats aparent and whats true..
    };

    // LOOKUP(SIMNET_DEVICE_REPORT, 10, "Mode")
    // LOOKUP(SIMNET_DEVICE_REPORT, 11, "Send Mode")
    //  start the ap controller ?
    void start()
    {
        m_scheduler.UpdateNextTime();
        // start timers
    }

    // todo run with task executor ?
    void process()
    {
        if (m_scheduler.IsTime())
        {
            sendStatus();
            sendMode();
            m_scheduler.UpdateNextTime();
        }
    }

private:
    tNMEA2000 &m_n2k;
    void sendStatus()
    {
        tN2kMsg msg;
        // 6 byte status msg
        msg.SetPGN(SIMRAD_AP_PGN);
        msg.Priority = 2;
        msg.AddByte(0);                                  // model AC 100 for NAC
        msg.AddByte(static_cast<uint8_t>(TYPE::Status)); //
        msg.AddByte(static_cast<uint8_t>(m_status));
        // msg.Add2ByteDouble(bridge.rudderAngle().N2k(), 0.0001);
        // msg.Add2ByteDouble(bridge.rudderAngle().N2k(), 0.0001);
        msg.AddByte(0xff); // Reserved
        msg.AddByte(0xff); // Reserved*/
        msg.AddByte(0xff); // Reserved*/

        m_n2k.SendMsg(msg, m_dev_id);
    }

    void sendMode()
    {
        tN2kMsg msg;
        // 6 byte status msg
        msg.SetPGN(SIMRAD_AP_PGN);
        msg.Priority = 2;
        msg.AddByte(0);                                // model AC 100 for NAC
        msg.AddByte(static_cast<uint8_t>(TYPE::Mode)); //
        msg.AddByte(static_cast<uint16_t>(m_mode));
        // msg.Add2ByteDouble(bridge.rudderAngle().N2k(), 0.0001);
        // msg.Add2ByteDouble(bridge.rudderAngle().N2k(), 0.0001);
        msg.AddByte(0xff); // Reserved
        msg.AddByte(0xff); // Reserved
        msg.AddByte(0xff); // Reserved
        msg.AddByte(0xff); // Reserved

        m_n2k.SendMsg(msg, m_dev_id);
    }

    tN2kSyncScheduler m_scheduler;

    uint32_t m_uid;
    int m_dev_id;

    Status m_status = Status::Manual;
    Mode m_mode = Mode::Standby;
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