#include "include/SimnetPGN.hpp"

// PGN6305 has many modes
//  Todo handle different modes in this object!! painful
void SetN2kPGN6305(tN2kMsg &msg, const SimnetApMsgType &mode, const PGN65305Payload &payload)
{
  msg.SetPGN(65305UL);
  msg.Priority = 7; // is this priority wrong ?
  // simrad id 1857 and 4 for marine,, //encodes as 419f
  // msg.Add2ByteUInt(1857 | (0x4 << (11 + 2)))

  msg.Add2ByteUInt(1857 | 0x8000 /*Maritime id*/ | 0x1800 /*reserved*/);
  msg.AddByte(0x0); // lets use id 0
  // heartbeat ? from https://github.com/canboat/canboat/issues/309
  // msg.AddByte(35);
  //  msg.AddByte(0); // this seems to allways be zero?
  msg.AddByte(static_cast<uint8_t>(mode));
  for (auto i = 0; i < 4; i++)
  {
    msg.AddByte(payload.data[i]);
  }
}
/*
void SetSimnetStatus(tN2kMsg &N2kMsg, const SimnetStatus &status)
{
}
*/
/*
SimnetCommand GetSimnetCommand(tN2kMsg &msg)
{
    if ()
}*/

// maybe a struct is a better option than parsing like this and a cast to the right cmd type!
void ParsePgn130850Ap(const tN2kMsg &N2kMsg, uint8_t &addr, SimnetCommand &simnetcmd, SimnetApStatus &apStatus, SimnetApCommand &apcmd, SimnetApDirection &dir, double &angle)
{
  int index = 0;
  uint16_t top = N2kMsg.Get2ByteUInt(index);
  // printf("Manufacturer %d\n", top & 0x7FF);
  // printf("Device Type %d\n", top >> 13 & 0x7);
  N2kMsg.GetByte(index);                                         // get reserved byte
  addr = N2kMsg.GetByte(index);                                  // BT-1 sends 254.. maybe im parsing bt1 messages wrong orca sends the Ap addr.
  simnetcmd = static_cast<SimnetCommand>(N2kMsg.GetByte(index)); // should be 255
  apStatus = static_cast<SimnetApStatus>(N2kMsg.GetByte(index)); // unsure if it matters what this is
  apcmd = static_cast<SimnetApCommand>(N2kMsg.GetByte(index));   // command to execute
  N2kMsg.GetByte(index);                                         // spare ?
  dir = static_cast<SimnetApDirection>(N2kMsg.GetByte(index));
  angle = N2kMsg.Get2ByteUDouble(0.0001, index);
  //+-1 0xae00 +-10 = 0xd106  //asume these are FP values in radians
  // tack port and tack starboard is 5b,3d //probably -60 or +60 ish in radians
}

void SetPGN65340(tN2kMsg &msg, const SimnetMode &mode /*Mode63540 &mode*/)
{
  // ac 12
  msg.SetPGN(65340UL);
  msg.Priority = 3; // according to https://github.com/htool/RaymarineAPtoFakeNavicoAutoPilot/blob/master/emulate.js#L454
  // simrad id 1857 and 4 for marine,, //encodes as 419f
  // uint16_t simnet_id = 1857;
  msg.Add2ByteUInt(1857 | 0x8000 /*Maritime id*/ | 0x1800 /*reserved*/);

  // msg.Add2ByteUInt(simnet_id | 0x8000);
  if (mode == SimnetMode::Standby)
  {
    msg.AddByte(0); // this seems to allways be zero?
  }
  else
  {
    // this is 16 as in automatic !!
    msg.AddByte(0x10); // active
  }

  switch (mode)
  {
  case SimnetMode::Standby:
    msg.AddByte(0x00);
    msg.AddByte(0xfe);
    msg.AddByte(0xf8);
    break;
  case SimnetMode::Heading:
    msg.AddByte(0x01);
    msg.AddByte(0xfe);
    msg.AddByte(0xfa);
    break;
  case SimnetMode::Followup: // this cant be right
  case SimnetMode::Wind:
    msg.AddByte(0x03);
    msg.AddByte(0xfe);
    msg.AddByte(0xfa);
    break;
  case SimnetMode::Navigation:
    msg.AddByte(0x03);
    msg.AddByte(0xfe);
    msg.AddByte(0xf8);
    break;
  }

  msg.AddByte(0x00);
  msg.AddByte(0x80);
}