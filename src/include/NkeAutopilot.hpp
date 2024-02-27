#pragma once

#include "NKETopline.hpp"

// todo see if there are more..
enum class FunctionCode : uint8_t
{
    WriteReg = 0xf4,
    ReadReg = 0xfc,
    ReadVersion = 0xf1,
};

#include <type_traits>

template <typename E>
constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

// isolated in case we want to make a controller web page or something that does more than just AP control
class Controller
{
public:
    // TODO record and understand how auto asign works
    Controller(tNKETopline &topline, uint8_t id = 0x02) : m_topline(topline), m_id(id)
    {
        topline.addMsgHandler([this](const Nke::_Message &msg)
                              { handleMessage(msg); });
        // Not exactly sure what to do here..
        // Do we add ourselves to the NkeTopline as a receiver..probably a good idea..
        // i guess a controller wants to listen to ALL the incoming data.. so basically turn filtering off ?
    }

    virtual void handleMessage(const Nke::_Message &msg) = 0;

private:
    tNKETopline &m_topline;
    // NkeHandler &m_handler;

protected:
    // should this be related to the controller at all somethow ?
    void write(uint8_t dest, uint8_t reg, uint16_t value)
    {
        Nke::_Message msg;
        msg.channel = to_integral(FunctionCode::WriteReg);
        msg.data[0] = dest;
        msg.data[1] = reg;
        msg.data[2] = ((value >> 8) & 0xFF);
        msg.data[3] = value & 0xff;
        msg.len = 4;
        m_topline.sendCommand(msg); // maybe the queue should live in the controller as its connected to its ID!
        // deal with the endians
        // m_topline.sendCommand(FunctionCode::WriteReg, dest, reg, {((value >> 8) & 0xFF), (value & 0xFF)});
        // maybe support little endian at some point
    }

    // reply is async
    void read(uint8_t dest, uint8_t reg /*, uint16_t &value*/)
    {
        Nke::_Message msg;
        msg.channel = to_integral(FunctionCode::ReadReg);
        msg.data[0] = dest;
        msg.data[1] = reg;
        msg.len = 2;
        m_topline.sendCommand(msg); // maybe the queue should live in the controller as its connected to its ID!

        // sleep this thread while waiting for response..
        // lazy but functional way to do it

        // m_topline.sendCommand(FunctionCode::ReadReg, dest, reg, {});
        //  send command but also catch response.. this is going to be a challange to complete as its async..!!
        //  for now ignore response
    }

    // todo make it so that read returns return values on completion ?
    // issue read with a cookie.. when cookie is returned call a callback
    uint8_t m_id;
};

enum class PilotMode : uint16_t
{
    Compass = 0x00,
    Aparent = 0x01,
    Rudder = 0x02,
    GPS = 0x03,
    HELM = 0x04,

    TrueWind = 0x06,
};

enum class PilotRegs : uint8_t
{
    ModeReg = 0x1,
    Target = 0x11,
};

// maybe control and mode needs two classes ..

enum class ControlRegs : uint8_t
{
    OnOffReg = 0x10,
};

enum class PilotEnabled : uint8_t
{
    True = 0x61,
    False = 0x60,
};

// the above could be coded as a bytestring with channel reg value as 4 bytes
namespace Nke
{
    class AutopilotController : public Controller
    {
    public:
        AutopilotController(tNKETopline &topline, NkeBridge &bridge, uint8_t controller_id = 0x2, uint8_t apMode = 0x4F, uint8_t apControl = 0x4e)
            : Controller(topline, controller_id), m_bridge(bridge), m_channel_mode(apMode), m_channel_control(apControl) {}

        void handleMessage(const Nke::_Message &msg) override
        {
            // async read replies
            if (msg.channel == 0xfc && msg.len == 4)
            {
                // pilot is responding on Control channel
                if (msg.data[0] == m_channel_control)
                {
                    switch (static_cast<ControlRegs>(msg.data[1]))
                    {
                    case ControlRegs::OnOffReg:
                        m_pilot_enabled = static_cast<PilotEnabled>(msg.data[2] << 8 | msg.data[3]);
                        break;
                    default:
                        Serial.printf("Unknown ControlReg %02x\n", msg.data[1]);
                    }
                    /*switch (msg.data[1])
                    {
                        case
                    }*/

                    // m_channel_control, to_integral(ControlRegs::OnOffReg)
                }

                if (msg.data[0] == m_channel_mode)
                {
                    switch (static_cast<PilotRegs>(msg.data[1]))
                    {
                    case PilotRegs::ModeReg:
                        // in theory only msg.data[3] affects this! but lets ignore that fact for now
                        m_mode = static_cast<PilotMode>(msg.data[2] << 8 | msg.data[3]);
                        break;
                    case PilotRegs::Target:
                        // read back current setting after setting it..
                        // we should compare but in this case we also get the ref when another controller sets it

                        m_target = static_cast<uint16_t>(msg.data[2] << 8 | msg.data[3]);
                        break;
                    default:
                        Serial.printf("Unknown PilotReg %02x\n", msg.data[1]);
                        break;
                    }
                }
            }

            if (msg.channel == static_cast<uint8_t>(Nke::Channel::PILOT_STATE))
            {
                // 0x5 idle, 0x3 GPS , 0 compass, 1 aparent wind, 2 rudder
                uint8_t pilot_state = msg.data[1];
            }
        }

        void setPilotMode(const PilotMode &mode)
        {
            // uint16_t ret_value;
            write(m_channel_mode, to_integral(PilotRegs::ModeReg), to_integral(mode));
            read(m_channel_mode, to_integral(PilotRegs::ModeReg));
        }

        // this requires read modify write!!
        void change(int change)
        {
            // this probably fails as some targets are %360..
            //  if (change < 0)
            //  {
            // m_target = m_target + change;

            switch (m_mode)
            {
            case PilotMode::Rudder:
                m_target = (m_target + 360 + change) % 360;
                break;
            }
            //   }
            /* else
             {
                 m_target = m_target + change;
             }*/
            write(m_channel_mode, to_integral(PilotRegs::Target), m_target);
            read(m_channel_mode, to_integral(PilotRegs::Target));
            /*   if (change < 0)
               {
                   subtract change
               }*/
            // read 0x11 and subtract valyue
        }

        void setGain(uint8_t gain)
        {
            if (gain < 9)
            {
                // apply
            }
        }

        uint8_t gain()
        {
        }

        void start()
        {
            // start autopilot... aparent is 33 ish deg
            // read mode
            // read mode
            // write aparent wind angle!! which is 305 (as in 305-360 => -55 ) //get this from NkeBridge data
            // veirfy aparent wind angle

            read(m_channel_mode, to_integral(PilotRegs::ModeReg));
            read(m_channel_mode, to_integral(PilotRegs::ModeReg));
            // about the time it should take to hit a frame and get the response need to run controller in dedicated thread!!
            // std::this_thread::sleep_for(100ms);
            vTaskDelay(100 / portTICK_PERIOD_MS);

            // uint16_t value = 0;
            switch (m_mode)
            {
            case PilotMode::Aparent:
                m_target = m_bridge.windAngle().Nke();
                break;
            case PilotMode::Rudder:
                m_target = m_bridge.rudderAngle().Nke() / 3;
                break;
            case PilotMode::Compass:
                m_target = m_bridge.heading().Nke();
                break;
            case PilotMode::GPS:
                m_target = m_bridge.btw().Nke();
                break;
            }
            // write it once and read back target..
            write(m_channel_mode, to_integral(PilotRegs::Target), m_target);
            read(m_channel_mode, to_integral(PilotRegs::Target));
            // write(m_channel_mode, to_integral(PilotRegs::Target), m_target);

            // get Ap state
            read(m_channel_control, to_integral(ControlRegs::OnOffReg));
            read(m_channel_control, to_integral(ControlRegs::OnOffReg));

            vTaskDelay(100 / portTICK_PERIOD_MS);
            // std::this_thread::sleep_for(100ms);

            write(m_channel_control, to_integral(ControlRegs::OnOffReg), to_integral(PilotEnabled::True));

            // this needs a function to give up
            uint8_t retries = 10;
            while (m_pilot_enabled != PilotEnabled::True && retries > 0)
            {
                read(m_channel_control, to_integral(ControlRegs::OnOffReg));
                vTaskDelay(100 / portTICK_PERIOD_MS);

                // std::this_thread::sleep_for(100ms);
                retries--;
            }

            // if (mode X get wind angle y etc.. we need to know the mode..)
            // switch()
            //

            // write(m_channel_mode, PilotRegs::Target, )
            /*
                        cmd 03 fc 4f code 01 00 01

                        cmd 03 fc 4f code 01 00 01

                        cmd 03 f4 4f code 11 01 31

                        cmd 03 fc 4f code 11 01 31

                        cmd 03 fc 4e code 10 00 60
                        cmd 03 fc 4e code 10 00 60
                        cmd 03 f4 4e code 10 00 61
                        cmd 03 fc 4e code 10 00 61
                        cmd 03 fc 4e code 10 00 61
                        cmd 03 fc 4e code 10 00 61

            */

            /*
            Rudder on off from controller!
    20:23:02.194 -> DEBUG Read: cmd fc  4f 01 00 02
     20:23:02.241 -> DEBUG Read: cmd fc  4f 01 00 02
     20:23:02.241 -> DEBUG write: cmd f4  4f 11 01 66
     20:23:02.272 -> DEBUG Read: cmd fc  4f 11 01 66
     20:23:02.272 -> Unknown PilotReg 11
     20:23:02.319 -> DEBUG Read: cmd fc  4e 10 00 60
     20:23:02.358 -> DEBUG Read: cmd fc  4e 10 00 60
     20:23:02.395 -> DEBUG write: cmd f4  4e 10 00 61
     20:23:02.427 -> DEBUG Read: cmd fc  4e 10 00 61
     20:23:02.474 -> DEBUG Read: cmd fc  4e 10 00 61
     20:23:02.520 -> DEBUG Read: cmd fc  4e 10 00 61
     20:23:04.655 -> DEBUG Read: cmd fc  4e 10 00 61
     20:23:04.700 -> DEBUG Read: cmd fc  4e 10 00 61
     20:23:04.700 -> DEBUG write: cmd f4  4e 10 00 60
     20:23:04.779 -> DEBUG Read: cmd fc  4e 10 00 60

            */
        }
        /*
        cmd   03 fc 4e code  10 00 61
        cmd   03 fc 4e code  10 00 61
        cmd   03 f4 4e code  10 00 60
        cmd   03 fc 4e code  10 00 60
        */
        void stop()
        {
            // read read write then read
            read(m_channel_control, to_integral(ControlRegs::OnOffReg));
            read(m_channel_control, to_integral(ControlRegs::OnOffReg));
            // guess if its off we dont continue here..
            write(m_channel_control, to_integral(ControlRegs::OnOffReg), to_integral(PilotEnabled::False));
            read(m_channel_control, to_integral(ControlRegs::OnOffReg)); // make sure it went off
        }

        // todo impl a get pilot mode

    private:
        NkeBridge &m_bridge;
        // need to see if you can support multiple controllers.. dont think so.
        // are these static in nke ?
        uint8_t m_channel_mode;
        uint8_t m_channel_control;
        PilotMode m_mode;
        uint16_t m_target; // the current control target
        PilotEnabled m_pilot_enabled;
    };
};