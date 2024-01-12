#pragma once

class NkeHandler
{
public:
    NkeHandler(uint8_t id) : m_id(id) {}

public:
    virtual void handle(const uint16_t &msg);
    uint8_t id() { return m_id; }

private:
    uint8_t m_id;
};