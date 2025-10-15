#ifndef LGFX_CONFIG_HPP
#define LGFX_CONFIG_HPP

#define LGFX_USE_V1

#include <LovyanGFX.hpp>
#include "pins.hpp"

// Adafruit ESP32-S3 Reverse TFT Feather display configuration
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

            // SPI bus configuration
            cfg.spi_host = TFT_SPI_HOST;
            cfg.spi_mode = TFT_SPI_MODE;
            cfg.freq_write = TFT_SPI_FREQ_WRITE;
            cfg.freq_read = TFT_SPI_FREQ_READ;
            cfg.spi_3wire = false;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;

            // Pin configuration
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
            cfg.offset_rotation = TFT_OFFSET_ROTATION;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.readable = false;
            cfg.invert = TFT_INVERT;
            cfg.rgb_order = TFT_RGB_ORDER;
            cfg.dlen_16bit = false;
            cfg.bus_shared = false;

            _panel_instance.config(cfg);
        }

        {
            auto cfg = _light_instance.config();

            cfg.pin_bl = TFT_PIN_BL;
            cfg.invert = false;
            cfg.freq = TFT_BL_FREQ;
            cfg.pwm_channel = TFT_BL_PWM_CHANNEL;

            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        setPanel(&_panel_instance);
    }
};

#endif // LGFX_CONFIG_HPP
