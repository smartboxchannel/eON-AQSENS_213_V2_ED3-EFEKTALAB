# EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3


Wireless, battery-operated Mini air quality and weather station. Air quality, temperature, humidity, pressure, weather forecast. Designed on the nRF52833, nRF52840 wireless radio modules. The temperature, humidity and pressure sensor BME280, air quality sensor SGP40 is used The device is powered by CR2477 battery. Power consumption when transmitting data is 8mA, in sleep mode is 35uA (see datasheet SGP40).

#### GDEH0213B72 (GDEH0213B73, Waveshare V2): https://ali.ski/05GbQ

(250x122,2.13inch E-Ink raw display panel Without PCB Communicate via SPI interface)

#### MS88SF3: https://ali.ski/I_UNg

(New FCC CE RoHs Certificated Compact (18.5×12.5×2mm) and Flexible ultra-low power wireless BLE 5.0 Module based on nRF52840 SoCs)

#### SGP40: https://ali.ski/eZQog

#### Don't donate to me, it doesn't work in this world: https://paypal.me/efektalab , just buy

#### Sale: 

Video: 

More info at http://efektalab.com/eink213_ed3

---
## Ver. A

![EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3 (MINI AIR QUALITY AND WEATHER STATION ON NRF52](https://github.com/smartboxchannel/EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3/blob/release-2.3a/IMAGES/a.png) 



![EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3 (MINI AIR QUALITY AND WEATHER STATION ON NRF52](https://github.com/smartboxchannel/EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3/blob/release-2.3a/IMAGES/IMG_20210524_220731.jpg) 


![EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3 (MINI AIR QUALITY AND WEATHER STATION ON NRF52](https://github.com/smartboxchannel/EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3/blob/release-2.3a/IMAGES/IMG_20210524_220528.jpg) 


![EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3 (MINI AIR QUALITY AND WEATHER STATION ON NRF52](https://github.com/smartboxchannel/EFEKTA-EINK213-AIR-QUALITY-SENSOR-NRF52840-V2-ED3/blobrelease-2.3a/IMAGES/005.jpg) 


---

![EFEKTA EINK102 TEMP HUM MICRO SENSOR NRF52 licenses](https://github.com/smartboxchannel/EFEKTA-EINK102-TEMP-HUM-MICRO-SENSOR-NRF52/blob/master/IMAGES/licenses.png)

---

### Instruction

#### First of all, I recommend installing the Arduino IDE portable (optional, but desirable)

https://www.arduino.cc/en/Guide/PortableIDE

#### 1. Install the latest arduino-nRF5 library (https://github.com/sandeepmistry/arduino-nRF5)

#### 2. Install the latest version of the MySensors library (https://github.com/mysensors/MySensors)

#### 3. Download the archive of this project to your computer

#### 4. Add support for devices of this project to the arduino-nRF5 library, the description is in the README.md file (https://github.com/smartboxchannel/EFEKTA-EINK213-TEMP-HUM-PRESS-SENSOR-NRF52840-V2/tree/master/for_sandeepmistry_nRF5/README.md)

#### 5. Add support for interrupts via gpiote, for this go to the ... packages \ sandeepmistry \ hardware \ nRF5 \ 0.7.0 \ cores \ nRF5 folder, and in the WInterrupts.c file, before the void GPIOTE_IRQHandler () function, add the line: \_\_attribute\_\_ ((weak ))

#### 6. Add the libraries in the archive () https://github.com/smartboxchannel/EFEKTA-EINK213-TEMP-HUM-PRESS-SENSOR-NRF52840-V2/tree/master/CODE/Arduino/libraries  of this project to the libraries folder on your computer ( path: ...\Documents\Arduino\libraries )

#### 7. Create an EINK213ED2 folder on your computer under the Arduino directory (Documents \ Arduino). Add the files of this project located in the Arduino section (https://github.com/smartboxchannel/EFEKTA-EINK213-TEMP-HUM-PRESS-SENSOR-NRF52840-V2/tree/master/CODE/Arduino) to the created EINK213ED2 folder

#### 8. Open the EINK213ED2.ino file in the Arduino IDE program, go to the aConfig.h tab and configure according to your board version and settings of your MySensors network.

#### 9. In the main menu of the Arduino IDE go to Tools-> Boards-> Nordic Semiconductors nRF5 Boards, in the list that opens, select the EFEKTA MWS213 V2 nRF52840 board. In the menu of the selected board, select the type of clock crystal (external), also select the Reset: Enable item.

#### 10. Click on the icon - check and then download

---

---

## Components (BOM):

SGP40: https://ali.ski/eZQog

MINEW MS88SF3 FCC CE RoHs Certificated wireless module nRF52840 SoCs - https://ali.ski/9LnWZ (1 pc - $7.75)

MAX44009EDT  - https://ali.ski/_PLrJ  (5 pcs - $9.3)
OR
MAX44009EDT  - https://ali.ski/bZoMnn  (1 pc - $1.9)

BME280 - https://ali.ski/wLEIir (2 pcs - $11.5)

Waveshare 250x122 2,13 - https://ali.ski/D4eOVU (1pc - $12.3)

FPC FFC 0.5mm connector socket 24 pin - https://ali.ski/lCBot0 (1 p - $0.77)

Cell Holder CR2450 - https://ali.ski/VdotsA (20 pcs - $3)

Micro Screws M1.4 3mm - https://ali.ski/gaFdO (100 pcs - $1)

Micro Screws M1.4 5mm - https://ali.ski/gaFdO (100 pcs - $1)

Micro SMD Tact Switch 2x4 2*4*3.5 - https://ali.ski/_D78Q (10 pcs - $1)

Neodymium magnet 15x3x3m - https://ali.ski/44p3R (10 pcs - $7)

Tantalum Capacitor 100UF 10V - https://ali.ski/Lx9iQd (10 pcs - $1,85)

Inductor Power Shielded Wirewound NR5040 5x5x4mm - https://ali.ski/iblu8q (50 pcs - $3)

Micro Button Tact Switch SMD 4Pin 3X4X2.5 - https://ali.ski/sGwFu (100 pcs - $1.7)

SMD Mini Toggle Slide Switch 7-Pin On/Off - https://ali.ski/xz9Yt (100 pcs - $3.2)

SI1308EDL - https://ali.ski/Q6Y12 (20 pcs - $5.3)
OR
Si1304BDL - https://ali.ski/938Klh (100 pcs - $15.5)

SMD Chip Multilayer Ceramic Capacitor 0603 1UF 25V - https://ali.ski/p3yr60 (100 pcs - $1.5)

SMD Chip Multilayer Ceramic Capacitor 0603 0.1UF 50V - https://ali.ski/p3yr60 (100 pcs - $1.5)

SMD Chip Multilayer Ceramic Capacitor 0805 4.7UF - https://ali.ski/iFXAc (100 pcs - $1.85)

MBR0530T1G - https://ali.ski/SJM7aK (100 pcs - $1.1)

SMD LED 0805 - https://ali.ski/wb6ZP (100 pcs - $2)

1% SMD resistor 0.47R 0603 - https://ali.ski/bX9HUg (100 pcs - $1.2)

1% SMD Resistor Kit Assorted Kit 1R-1M 0603 -  https://ali.ski/npItF (660 pcs - $1.45)


SUNLU PLA Carbon Fiber Premium 3D Printer Filament - https://ali.ski/bQkNR (1 pcs - $26.5)

405nm UV Resin For 3d Printer - https://ali.ski/sW0MUm (1 pcs - $18.5)
