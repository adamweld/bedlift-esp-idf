#ifndef LGFX_CONFIG_HPP
#define LGFX_CONFIG_HPP

#define LGFX_USE_V1

#include <LovyanGFX.hpp>

// Adafruit ESP32-S3 Reverse TFT Feather display (ST7789 240x135)
#define TFT_PIN_SCLK  36
#define TFT_PIN_MOSI  35
#define TFT_PIN_MISO  37
#define TFT_PIN_DC    40
#define TFT_PIN_CS    42
#define TFT_PIN_RST   41
#define TFT_PIN_BL    45
#define TFT_PANEL_WIDTH     135
#define TFT_PANEL_HEIGHT    240
#define TFT_OFFSET_X        52
#define TFT_OFFSET_Y        40
#define TFT_SCREEN_ROTATION 3

class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Light_PWM _light_instance;

public:
    LGFX(void)
    {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000;
            cfg.freq_read = 16000000;
            cfg.spi_3wire = false;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = TFT_PIN_SCLK;
            cfg.pin_mosi = TFT_PIN_MOSI;
            cfg.pin_miso = TFT_PIN_MISO;
            cfg.pin_dc = TFT_PIN_DC;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs = TFT_PIN_CS;
            cfg.pin_rst = TFT_PIN_RST;
            cfg.pin_busy = -1;
            cfg.panel_width = TFT_PANEL_WIDTH;
            cfg.panel_height = TFT_PANEL_HEIGHT;
            cfg.offset_x = TFT_OFFSET_X;
            cfg.offset_y = TFT_OFFSET_Y;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.readable = false;
            cfg.invert = true;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = false;
            _panel_instance.config(cfg);
        }
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl = TFT_PIN_BL;
            cfg.invert = false;
            cfg.freq = 44100;
            cfg.pwm_channel = 1;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }
        setPanel(&_panel_instance);
    }
};

#endif // LGFX_CONFIG_HPP
