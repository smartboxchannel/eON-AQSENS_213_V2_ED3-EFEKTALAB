
// ###################   Mini air quality and wither station with electronic ink display 2.13 Inch | nRF52   ############### //
//                                                                                                                           //
//        @filename   :   MWS213_V2R3_ED3.ino                                                                                //
//        @brief en   :   Wireless, battery-operated air quality(VOC Sensor SGP40), temperature,humidity and pressure        //
//                        sensor(BME280) with electronic ink display(Good Display GDEH0213B72, GDEH0213B73, Waveshare V2).   //
//                        Works on SOC nRF52.                                                                                //
//        @brief ru   :   Беcпроводной, батарейный датчик качества воздуха (ЛОС сенсорс SGP40), температуры, влажности       //
//                        и давления(BME280) с дисплеем на электронных чернилах(Good Display GDEH0213B72, GDEH0213B73,       //
//                        Waveshare V2).                                                                                     //
//                        Работает на nRF52832, nRF52840.                                                                    //
//                                                                                                                           //
//        @author     :   Andrew Lamchenko aka Berk                                                                          //
//                                                                                                                           //
//        Copyright (C) 2021, EFEKTALAB                                                                                      //
//        Copyright (c) 2020, Sensirion AG                                                                                   //
//        Copyright (c) 2020, Bosch Sensortec GmbH. All rights reserved.                                                     //
//        Copyright (c) 2014-2015, Arduino LLC.  All right reserved.                                                         //
//        Copyright (c) 2016, Arduino Srl.  All right reserved.                                                              //
//        Copyright (c) 2017, Sensnology AB. All right reserved.                                                             //
//        Copyright (C) 2020, Waveshare                                                                                      //
//                                                                                                                           //
// ######################################################################################################################### //

/*****************************************************************************
  | File      	:   epd2in13_V2.cpp
  | Author      :   Waveshare team
  | Function    :   2.13inch e-paper V2
  | Info        :
  ----------------
  |	This version:   V1.0
  | Date        :   2019-06-24
  | Info        :
  #
  # Permission is hereby granted, free of charge, to any person obtaining a copy
  # of this software and associated documnetation files (the "Software"), to deal
  # in the Software without restriction, including without limitation the rights
  # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  # copies of the Software, and to permit persons to  whom the Software is
  # furished to do so, subject to the following conditions:
  #
  # The above copyright notice and this permission notice shall be included in
  # all copies or substantial portions of the Software.
  #
  # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  # FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  # LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  # THE SOFTWARE.
  #
******************************************************************************/
#include <stdlib.h>
#include "eink213_V2.h"

const unsigned char lut_full_update[] = {
  0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,       //LUT0: BB:     VS 0 ~7
  0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,       //LUT1: BW:     VS 0 ~7
  0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00,       //LUT2: WB:     VS 0 ~7
  0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00,       //LUT3: WW:     VS 0 ~7
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       //LUT4: VCOM:   VS 0 ~7

  0x03, 0x03, 0x00, 0x00, 0x02,                   // TP0 A~D RP0
  0x09, 0x09, 0x00, 0x00, 0x02,                   // TP1 A~D RP1
  0x03, 0x03, 0x00, 0x00, 0x02,                   // TP2 A~D RP2
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP3 A~D RP3
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP4 A~D RP4
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP5 A~D RP5
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP6 A~D RP6

  0x15, 0x41, 0xA8, 0x32, 0x30, 0x0A,
};

const unsigned char lut_partial_update[] = { //20 bytes
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       //LUT0: BB:     VS 0 ~7
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       //LUT1: BW:     VS 0 ~7
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       //LUT2: WB:     VS 0 ~7
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       //LUT3: WW:     VS 0 ~7
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,       //LUT4: VCOM:   VS 0 ~7

  0x0A, 0x00, 0x00, 0x00, 0x00,                   // TP0 A~D RP0
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP1 A~D RP1
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP2 A~D RP2
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP3 A~D RP3
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP4 A~D RP4
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP5 A~D RP5
  0x00, 0x00, 0x00, 0x00, 0x00,                   // TP6 A~D RP6

  0x15, 0x41, 0xA8, 0x32, 0x30, 0x0A,
};

Epd::~Epd()
{
};

Epd::Epd()
{
  reset_pin = RST_PIN;
  dc_pin = DC_PIN;
  cs_pin = CS_PIN;
  busy_pin = BUSY_PIN;
  width = EPD_WIDTH;
  height = EPD_HEIGHT;
};

/**
    @brief: basic function for sending commands
*/
void Epd::SendCommand(unsigned char command)
{
  DigitalWrite(dc_pin, LOW);
  SpiTransfer(command);
}

/**
    @brief: basic function for sending data
*/
void Epd::SendData(unsigned char data)
{
  DigitalWrite(dc_pin, HIGH);
  SpiTransfer(data);
}

/**
    @brief: Wait until the busy_pin goes HIGH
*/
void Epd::WaitUntilIdle(void)
{
  while (1) {     //LOW: idle, HIGH: busy
    if (DigitalRead(busy_pin) == 0)
      break;
    DelayMs(100);
  }
}

int Epd::Init(char Mode)
{
  /* this calls the peripheral hardware interface, see epdif */
  if (IfInit() != 0) {
    return -1;
  }

  Reset();

  int count;
  if (Mode == FULL) {
    WaitUntilIdle();
    SendCommand(0x12); // soft reset
    WaitUntilIdle();

    SendCommand(0x74); //set analog block control
    SendData(0x54);
    SendCommand(0x7E); //set digital block control
    SendData(0x3B);

    SendCommand(0x01); //Driver output control
    SendData(0xF9);
    SendData(0x00);
    SendData(0x00);

    SendCommand(0x11); //data entry mode
    SendData(0x01);

    SendCommand(0x44); //set Ram-X address start/end position
    SendData(0x00);
    SendData(0x0F);    //0x0C-->(15+1)*8=128

    SendCommand(0x45); //set Ram-Y address start/end position
    SendData(0xF9);   //0xF9-->(249+1)=250
    SendData(0x00);
    SendData(0x00);
    SendData(0x00);

    SendCommand(0x3C); //BorderWavefrom
    SendData(0x03);

    SendCommand(0x2C); //VCOM Voltage
    SendData(0x55); //

    SendCommand(0x03);
    SendData(lut_full_update[70]);

    SendCommand(0x04); //
    SendData(lut_full_update[71]);
    SendData(lut_full_update[72]);
    SendData(lut_full_update[73]);

    SendCommand(0x3A);     //Dummy Line
    SendData(lut_full_update[74]);
    SendCommand(0x3B);     //Gate time
    SendData(lut_full_update[75]);

    SendCommand(0x32);
    for (count = 0; count < 70; count++) {
      SendData(lut_full_update[count]);
    }

    SendCommand(0x4E);   // set RAM x address count to 0;
    SendData(0x00);
    SendCommand(0x4F);   // set RAM y address count to 0X127;
    SendData(0xF9);
    SendData(0x00);
    WaitUntilIdle();
  } else if (Mode == PART) {
    SendCommand(0x2C);     //VCOM Voltage
    SendData(0x26);

    WaitUntilIdle();

    SendCommand(0x32);
    for (count = 0; count < 70; count++) {
      SendData(lut_partial_update[count]);
    }

    SendCommand(0x37);
    SendData(0x00);
    SendData(0x00);
    SendData(0x00);
    SendData(0x00);
    SendData(0x40);
    SendData(0x00);
    SendData(0x00);

    SendCommand(0x22);
    SendData(0xC0);

    SendCommand(0x20);
    WaitUntilIdle();

    SendCommand(0x3C); //BorderWavefrom
    SendData(0x01);
  } else {
    return -1;
  }

  return 0;
}

void Epd::SetWindows(int x_start, int y_start, int x_end, int y_end)
{
  SendCommand(0x44);
  /* x point must be the multiple of 8 or the last 3 bits will be ignored */
  SendData((x_start >> 3) & 0xFF);
  SendData((x_end >> 3) & 0xFF);
  SendCommand(0x45);
  SendData(y_start & 0xFF);
  SendData((y_start >> 8) & 0xFF);
  SendData(y_end & 0xFF);
  SendData((y_end >> 8) & 0xFF);
}

/**
    @brief: private function to specify the start point for data R/W
*/
void Epd::SetCursor(int x, int y)
{
  SendCommand(0x4E);
  /* x point must be the multiple of 8 or the last 3 bits will be ignored */
  SendData((x >> 3) & 0xFF);
  SendCommand(0X4F);
  SendData(y & 0xFF);
  SendData((y >> 8) & 0xFF);
  WaitUntilIdle();
}


void Epd::DisplayPartWindows(const unsigned char* frame_buffer, uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
  uint16_t Width, Height;
  Width = ((Xend - Xstart) % 8 == 0) ? ((Xend - Xstart) / 8 ) : ((Xend - Xstart) / 8 + 1);
  Height = Yend - Ystart;

  SetWindows(Xstart, Ystart, Xend, Yend);
  uint16_t i, j;
  for (j = 0; j < Height; j++) {
    SetCursor(Xstart, Ystart + j);
    SendCommand(0x24);
    for (i = 0; i < Width; i++) {
      SendData(frame_buffer[i + j * Width]);
    }
  }

  for (j = 0; j < Height; j++) {
    SetCursor(Xstart, Ystart + j);
    SendCommand(0x26);
    for (i = 0; i < Width; i++) {
      SendData(~frame_buffer[i + j * Width]);
    }
  }
}


void Epd::Display(const unsigned char* frame_buffer,  char mode)
{
  int w = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
  int h = EPD_HEIGHT;

  if (frame_buffer != NULL) {
    SendCommand(0x24);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(frame_buffer[i + j * w]);
      }
    }
  }

  if (frame_buffer != NULL) {
    SendCommand(0x26);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(~frame_buffer[i + j * w]);
      }
    }
  }

  if (mode == FULL) {
    //DISPLAY REFRESH
    SendCommand(0x22);
    SendData(0xC7);
    SendCommand(0x20);
    WaitUntilIdle();
  } else if (mode == PART) {
    //DISPLAY REFRESH
    SendCommand(0x22);
    SendData(0x0C);
    SendCommand(0x20);
    WaitUntilIdle();
  }

}


/**
    @brief: module reset.
            often used to awaken the module in deep sleep,
            see Epd::Sleep();
*/
void Epd::Reset(void)
{
  DigitalWrite(reset_pin, HIGH);
  DelayMs(200);
  DigitalWrite(reset_pin, LOW);                //module reset
  DelayMs(10);
  DigitalWrite(reset_pin, HIGH);
  DelayMs(200);
}


void Epd::Clear(bool mode, char mode2)
{
  if (mode == false) {
    int w, h;
    w = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
    h = EPD_HEIGHT;
    SendCommand(0x26);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(0x00);
      }
    }

    SendCommand(0x24);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(0xff);
      }
    }

    if (mode2 == FULL) {
      //DISPLAY REFRESH
      SendCommand(0x22);
      SendData(0xC7);
      SendCommand(0x20);
      WaitUntilIdle();
    } else if (mode2 == PART) {
      //DISPLAY REFRESH
      SendCommand(0x22);
      SendData(0x0C);
      SendCommand(0x20);
      WaitUntilIdle();
    }

  } else if (mode == true) {
    int w, h;
    w = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
    h = EPD_HEIGHT;
    SendCommand(0x26);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(0xff);
      }
    }

    SendCommand(0x24);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(0x00);
      }
    }

    if (mode2 == FULL) {
      //DISPLAY REFRESH
      SendCommand(0x22);
      SendData(0xC7);
      SendCommand(0x20);
      WaitUntilIdle();
    } else if (mode2 == PART) {
      //DISPLAY REFRESH
      SendCommand(0x22);
      SendData(0x0C);
      SendCommand(0x20);
      WaitUntilIdle();
    }

  }
}


void Epd::DisplayWrite(char Mode) {
  if (Mode == FULL) {
    //DISPLAY REFRESH
    SendCommand(0x22);
    SendData(0xC7);
    SendCommand(0x20);
    WaitUntilIdle();
  } else if (Mode == PART) {
    //DISPLAY REFRESH
    SendCommand(0x22);
    SendData(0x0C);
    SendCommand(0x20);
    WaitUntilIdle();
  }
  //return 0;
}



void Epd::DisplayPartBaseImage(const unsigned char* frame_buffer)
{
  int w = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
  int h = EPD_HEIGHT;

  if (frame_buffer != NULL) {
    SendCommand(0x26);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(pgm_read_byte(&frame_buffer[i + j * w]));
      }
    }

    SendCommand(0x24);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(pgm_read_byte(&frame_buffer[i + j * w]));
      }
    }
  }
}

void Epd::DisplayPart(const unsigned char* frame_buffer)
{
  int w = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8 ) : (EPD_WIDTH / 8 + 1);
  int h = EPD_HEIGHT;

  if (frame_buffer != NULL) {
    SendCommand(0x24);
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        SendData(pgm_read_byte(&frame_buffer[i + j * w]));
      }
    }
  }

  //DISPLAY REFRESH
  SendCommand(0x22);
  SendData(0x0C);
  SendCommand(0x20);
  WaitUntilIdle();
}



/**
    @brief: After this command is transmitted, the chip would enter the
            deep-sleep mode to save power.
            The deep sleep mode would return to standby by hardware reset.
            The only one parameter is a check code, the command would be
            executed if check code = 0xA5.
            You can use Epd::Init() to awaken
*/
void Epd::Sleep()
{
  SendCommand(0x10); //enter deep sleep
  SendData(0x01);
  DelayMs(100);
}

/* END OF FILE */
