
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

extern const unsigned char Especially[];
extern const unsigned char IMAGE_LOGO4[];
extern const unsigned char IMAGE_LOGO5[];
extern const unsigned char IMAGE_LOGO6[];
extern const unsigned char IMAGE_DATA_B100[];
extern const unsigned char IMAGE_DATA_B87[];
extern const unsigned char IMAGE_DATA_B75[];
extern const unsigned char IMAGE_DATA_B63[];
extern const unsigned char IMAGE_DATA_B50[];
extern const unsigned char IMAGE_DATA_B38[];
extern const unsigned char IMAGE_DATA_B25[];
extern const unsigned char IMAGE_DATA_B13[];
extern const unsigned char IMAGE_DATA_B0[];
extern const unsigned char IMAGE_DATA_L100[];
extern const unsigned char IMAGE_DATA_L80[];
extern const unsigned char IMAGE_DATA_L60[];
extern const unsigned char IMAGE_DATA_L40[];
extern const unsigned char IMAGE_DATA_L20[];
extern const unsigned char IMAGE_DATA_L0[];
extern const unsigned char IMAGE_DATA_W0[];
extern const unsigned char IMAGE_DATA_W1[];
extern const unsigned char IMAGE_DATA_W2[];
extern const unsigned char IMAGE_DATA_W3[];
extern const unsigned char IMAGE_DATA_W31[];
extern const unsigned char IMAGE_DATA_W4[];
extern const unsigned char IMAGE_DATA_W5[];
extern const unsigned char IMAGE_DATA_EW0[];
extern const unsigned char IMAGE_DATA_EW1[];
extern const unsigned char IMAGE_DATA_EW2[];
extern const unsigned char IMAGE_DATA_EW3[];
extern const unsigned char IMAGE_DATA_EW31[];
extern const unsigned char IMAGE_DATA_EW4[];
extern const unsigned char IMAGE_DATA_EW5[];
extern const unsigned char IMAGE_DATA_NN9[];
extern const unsigned char IMAGE_DATA_NN8[];
extern const unsigned char IMAGE_DATA_NN7[];
extern const unsigned char IMAGE_DATA_NN6[];
extern const unsigned char IMAGE_DATA_NN5[];
extern const unsigned char IMAGE_DATA_NN4[];
extern const unsigned char IMAGE_DATA_NN3[];
extern const unsigned char IMAGE_DATA_NN2[];
extern const unsigned char IMAGE_DATA_NN1[];
extern const unsigned char IMAGE_DATA_NN0[];
extern const unsigned char IMAGE_DATA_NNS0[];
extern const unsigned char IMAGE_DATA_NNS1[];
extern const unsigned char IMAGE_DATA_NNS2[];
extern const unsigned char IMAGE_DATA_NNS3[];
extern const unsigned char IMAGE_DATA_NNS4[];
extern const unsigned char IMAGE_DATA_NNS5[];
extern const unsigned char IMAGE_DATA_NNS6[];
extern const unsigned char IMAGE_DATA_NNS7[];
extern const unsigned char IMAGE_DATA_NNS8[];
extern const unsigned char IMAGE_DATA_NNS9[];
extern const unsigned char IMAGE_DATA_NNPOINT[];
extern const unsigned char IMAGE_DATA_NNC[];
extern const unsigned char IMAGE_DATA_NNF[];
extern const unsigned char IMAGE_DATA_NNSS0[];
extern const unsigned char IMAGE_DATA_NNSS1[];
extern const unsigned char IMAGE_DATA_NNSS2[];
extern const unsigned char IMAGE_DATA_NNSS3[];
extern const unsigned char IMAGE_DATA_NNSS4[];
extern const unsigned char IMAGE_DATA_NNSS5[];
extern const unsigned char IMAGE_DATA_NNSS6[];
extern const unsigned char IMAGE_DATA_NNSS7[];
extern const unsigned char IMAGE_DATA_NNSS8[];
extern const unsigned char IMAGE_DATA_NNSS9[];
extern const unsigned char IMAGE_DATA_NNSSS0[];
extern const unsigned char IMAGE_DATA_NNSSS1[];
extern const unsigned char IMAGE_DATA_NNSSS2[];
extern const unsigned char IMAGE_DATA_NNSSS3[];
extern const unsigned char IMAGE_DATA_NNSSS4[];
extern const unsigned char IMAGE_DATA_NNSSS5[];
extern const unsigned char IMAGE_DATA_NNSSS6[];
extern const unsigned char IMAGE_DATA_NNSSS7[];
extern const unsigned char IMAGE_DATA_NNSSS8[];
extern const unsigned char IMAGE_DATA_NNSSS9[];
extern const unsigned char IMAGE_DATA_NNSSPOINT[];
extern const unsigned char IMAGE_DATA_NNSSSPOINT[];
extern const unsigned char IMAGE_DATA_HHPR[];
extern const unsigned char IMAGE_DATA_PPMM[];
extern const unsigned char IMAGE_DATA_PPGPA[];
extern const unsigned char IMAGE_CON[];
extern const unsigned char IMAGE_ENCON[];
extern const unsigned char IMAGE_RBATT[648];
extern const unsigned char IMAGE_ERBATT[648];
extern const unsigned char IMAGE_CONF[648];
extern const unsigned char IMAGE_ECONF[648];
extern const unsigned char IMAGE_SEARCH[648];
extern const unsigned char IMAGE_ESEARCH[648];
extern const unsigned char IMAGE_RTIME[648];
extern const unsigned char IMAGE_ERTIME[648];
extern const unsigned char IMAGE_PAIR[648];
extern const unsigned char IMAGE_EPAIR[648];
extern const unsigned char IMAGE_RESET[648];
extern const unsigned char IMAGE_ERESET[648];
extern const unsigned char IMAGE_PRESENT[648];
extern const unsigned char IMAGE_EPRESENT[648];
extern const unsigned char IMAGE_ACTIV[216];
extern const unsigned char IMAGE_EACTIV[216];
extern const unsigned char IMAGE_UP[];
extern const unsigned char IMAGE_DOWN[];
extern const unsigned char IMAGE_COLOR[];
extern const unsigned char IMAGE_ECOLOR[];
extern const unsigned char IMAGE_CONACTIV[];
extern const unsigned char IMAGE_ECONACTIV[];
extern const unsigned char IMAGE_NOCONACTIV[];
extern const unsigned char IMAGE_ENOCONACTIV[];
extern const unsigned char LOC[];
extern const unsigned char VOC[];
extern const unsigned char AQ[];
extern const unsigned char AQEN[];
extern const unsigned char AQST0[];
extern const unsigned char AQST0EN[];
extern const unsigned char AQST1[];
extern const unsigned char AQST1EN[];
extern const unsigned char AQST2[];
extern const unsigned char AQST2EN[];
extern const unsigned char AQST3[];
extern const unsigned char AQST3EN[];
extern const unsigned char AQST4[];
extern const unsigned char AQST4EN[];
extern const unsigned char AQST5[];
extern const unsigned char AQST5EN[];
extern const unsigned char AQST6[];
extern const unsigned char AQST6EN[];
extern const unsigned char NOSAVE[];
extern const unsigned char NOSAVE1[];
extern const unsigned char NOSAVE2[];
extern const unsigned char NOSAVE3[];
extern const unsigned char NOSAVE4[];
extern const unsigned char NOSAVE5[];
extern const unsigned char NOSAVE6[];
extern const unsigned char NOSAVE7[];
extern const unsigned char NOSAVE8[];
extern const unsigned char NOSAVE9[];
extern const unsigned char NOSAVE10[];
extern const unsigned char NOSAVE11[];
extern const unsigned char NOSAVE12[];
extern const unsigned char SAVE[];
extern const unsigned char LOAD[];

/* FILE END */
