#pragma once

#include <M5Unified.h>

// Make more generic in the future maybe ?
namespace Nke
{
    class Display
    {
    public:
        Display(M5GFX &display, const String &heading)
            : m_disp(display), m_head(heading)
        {
            m_disp.clear();
            drawSurface();
        }

        void update(const String &info)
        {
            m_disp.startWrite();

            m_disp.setTextDatum(MC_DATUM);
            // m_disp.setFont(&fonts::FreeMonoBold24pt7b);
            // m_disp.setFont(&fonts::Font8)
            //   Where do we set it ?
            // m_disp.setTextSize(10);
            // m_disp.setCursor(0, 0);
            // font is zero
            // need to work out a good position
            // void setClipRect(int32_t x, int32_t y, int32_t w, int32_t h)
            // setClipRect()
            // m_disp.clearClo
            // m_disp.drawString("      ", width / 2, topSize + ((height - topSize) / 2), &fonts::Font8);
            if (m_data.length() != info.length())
            {
                // clear this part of the display
                m_disp.fillRect(2, topSize + 2, width - 4, height - topSize - 4, 0x0000);

                // m_disp.drawString("      ", width / 2, topSize + ((height - topSize) / 2), &fonts::Font8);
                m_data = info;
            }
            // m_disp.drawString(info, width / 2, topSize + ((height - topSize) / 2), &fonts:: Font8x8C64);
            // m_disp.setTextSize(5);
            m_disp.drawString(info, width / 2, topSize + ((height - topSize) / 2), &fonts::AsciiFont24x48);

            // m_disp.println(info);
            m_disp.endWrite();
        }

    private:
        void drawSurface()
        {
            // m_disp.setFont(&fonts::Roboto_Thin_24);
            m_disp.setFont(&fonts::FreeSerifBold24pt7b);
            m_disp.setTextSize(1);
            m_disp.setTextDatum(MC_DATUM);
            m_disp.drawString(m_head, width / 2, topSize / 2);
            m_disp.drawRect(0, 0, width, topSize, 0xff80);
            m_disp.drawRect(0, (topSize + 1), width, height - (topSize + 1), 0x001F);
        }
        M5GFX &m_disp;
        String m_head;
        String m_data;
        static constexpr auto topSize = 50;
        static constexpr auto width = 320;
        static constexpr auto height = 240;
    };
};