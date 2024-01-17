#pragma once

namespace Nke
{
    // standard message from nke bus
    static constexpr int MaxNkeMessageSize = 6;
    struct _Message
    {
        // id can be fc or bc etc as well!
        uint8_t channel; // might need to bump this to 16 bits //nke calls this channels in manuals
        uint8_t len;
        uint8_t data[MaxNkeMessageSize]; // should be able to contain even fc commands
    };

    // wrapper class over _message with functions ??
    /*class Message : public _Message
    {

    };*/
}
using tNkeMsg = Nke::_Message;