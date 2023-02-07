/**
 **************************************************
 *
 * @file        Generic-easyC-SOLDERED.h
 * @brief       Header file for sensor specific code.
 *
 *
 * @copyright GNU General Public License v3.0
 * @authors     @ soldered.com
 ***************************************************/

#ifndef __TFT_LCD_2_8_TOUCH_SOLDERED__
#define __TFT_LCD_2_8_TOUCH_SOLDERED__

#include "Arduino.h"

class Sensor
{
  public:
    Sensor(int _pin);

  protected:
    void initializeNative();

  private:
    int pin;
};

#endif
