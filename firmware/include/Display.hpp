#ifdef USE_DISPLAY
#ifndef __DISPLAY_HPP__
#define __DISPLAY_HPP__

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET 28    // 4 // Reset pin # (or -1 if sharing Arduino reset pin)

class Display
{
public:
    Display();
    void clear();
    void text(const String &data);
    void text(const char *data);
    void text(int data);
    void setup();

private:
    Adafruit_SSD1306 ssd1306_;
};

Display::Display() : ssd1306_(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

inline void Display::clear()
{
    ssd1306_.clearDisplay();
    ssd1306_.setCursor(0, 0);
}

inline void Display::text(const String &data)
{
    clear();
    ssd1306_.println(data);
    ssd1306_.display();
}

inline void Display::text(const char *data)
{
    clear();
    ssd1306_.println(data);
    ssd1306_.display();
}

inline void Display::text(int data)
{
    clear();
    ssd1306_.println(data);
    ssd1306_.display();
}

inline void Display::setup()
{
    if (!ssd1306_.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        return;
    }
    ssd1306_.clearDisplay();
    ssd1306_.setTextSize(1);              // Normal 1:1 pixel scale
    ssd1306_.setTextColor(SSD1306_WHITE); // Draw white text
    ssd1306_.cp437(true);                 // Use full-256-char 'Code Page 437' font
    text("Starting...\nDogBot Firmware!");
}

#endif // __DISPLAY_HPP__
#endif // USE_DISPLAY