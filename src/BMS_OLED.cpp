#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include "globals.h"
#include "Logger.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, -1);

bool oledConnected = false;

bool clearNextUpdate = false;
/**
 * @brief Sets up the OLED display if it's connected.
 *
 * This function will log if the display is detected or not, and set the
 * oledConnected flag accordingly. If the display is connected, it will
 * clear the display, set the cursor to the top left, set the text size
 * to 1, and set the text color to white. Finally, it will print "Loading..."
 * and display it.
 *
 * @note The display is set to address 0x3C for a 128x32 display.
 */
void setupOLED()
{
    Wire2.begin();

    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3C for 128x32
        Logger::info("OLED Display not detected");
        oledConnected = false;
    }
    else
    {
        Logger::info("OLED Display detected");
        oledConnected = true;
    }
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.print("Loading...");
    oled.setCursor(0, 3);
    oled.print("_____________________");
    oled.display();
}

/**
 * @brief Updates the OLED display with the current status.
 *
 * This function will take a pointer to a null terminated char array and
 * display it on the OLED screen. The word "STATUS" will be written on the
 * top line of the screen, underlined, and the passed in text will be written
 * on the lines below.
 *
 * @param statusText A pointer to a null terminated char array
 */
void updateOLEDStatus(char *statusText)
{
    if (oledConnected)
    {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(WHITE);
        oled.setCursor(0, 0);
        oled.print("STATUS");
        // Underline the text
        oled.drawLine(0, 9, 128, 9, WHITE);
        oled.setCursor(0, 10);
        oled.print(statusText);
        oled.display();
    }
}

void updateOLEDError(char *errorText)
{
    if (oledConnected)
    {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(WHITE);
        oled.setCursor(0, 0);
        oled.print("ERROR");
        // Underline the text
        oled.drawLine(0, 9, 128, 9, WHITE);
        oled.setCursor(0, 10);
        oled.print(errorText);
        oled.display();
    }
}


void oledPrint(const char *newStatusText)
{
    if (oledConnected)
    {
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.setTextSize(1);
        oled.setTextColor(WHITE);
        oled.print(newStatusText);
        oled.display();
    }
}