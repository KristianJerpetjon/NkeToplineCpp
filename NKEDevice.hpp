

class NkeDevice
{
  //TODO handle multiple id's
  NkeDevice(uint32_t id) : m_id(id) {

  }

  uint8_t id() { return m_id; }

  private:
    uint32_t m_id;

}