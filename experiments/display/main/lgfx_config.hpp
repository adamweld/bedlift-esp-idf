#ifndef LGFX_CONFIG_HPP
#define LGFX_CONFIG_HPP

#define LGFX_USE_V1

#include <LovyanGFX.hpp>

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

            // SPI bus configuration for ESP32-S3 Reverse TFT
            cfg.spi_host = SPI2_HOST;     // Use SPI2
            cfg.spi_mode = 0;             // SPI mode 0
            cfg.freq_write = 40000000;    // SPI frequency when writing (40MHz - reduced for stability)
            cfg.freq_read = 16000000;     // SPI frequency when reading
            cfg.spi_3wire = false;        // Use 4-wire SPI
            cfg.use_lock = true;          // Use transaction lock
            cfg.dma_channel = SPI_DMA_CH_AUTO; // Use DMA

            // Pin configuration for Adafruit ESP32-S3 Reverse TFT
            cfg.pin_sclk = 36;            // SPI SCLK (GPIO36 - SCK)
            cfg.pin_mosi = 35;            // SPI MOSI (GPIO35 - MOSI)
            cfg.pin_miso = 37;            // SPI MISO (GPIO37 - MISO, not used for write-only)
            cfg.pin_dc = 40;              // Data/Command pin (GPIO40 - TFT_DC)

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();

            cfg.pin_cs = 42;              // Chip select
            cfg.pin_rst = 41;             // Reset pin
            cfg.pin_busy = -1;            // Busy pin (not used)

            cfg.panel_width = 135;        // Actual pixel width
            cfg.panel_height = 240;       // Actual pixel height
            cfg.offset_x = 52;            // X offset (ST7789 240x320 driver with 135x240 panel)
            cfg.offset_y = 40;            // Y offset
            cfg.offset_rotation = 0;      // Rotation offset
            cfg.dummy_read_pixel = 8;     // Dummy read bits before pixel read
            cfg.dummy_read_bits = 1;      // Dummy read bits before non-pixel read
            cfg.readable = false;         // Data read is not supported
            cfg.invert = true;            // Invert panel brightness
            cfg.rgb_order = false;         // RGB order (false = RGB, true = BGR)
            cfg.dlen_16bit = false;       // Data length 16-bit units
            cfg.bus_shared = false;       // Bus shared with other devices

            _panel_instance.config(cfg);
        }

        {
            auto cfg = _light_instance.config();

            cfg.pin_bl = 45;              // Backlight pin
            cfg.invert = false;           // Backlight invert
            cfg.freq = 44100;             // PWM frequency
            cfg.pwm_channel = 1;          // PWM channel

            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        setPanel(&_panel_instance);
    }
};

#endif // LGFX_CONFIG_HPP
