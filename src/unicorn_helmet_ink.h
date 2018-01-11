/************************************************************************************
   BUSY->D3(MISO),
   RST->D7,
   DC->D6,
   CS->D5(SS),
   CLK->D4(SCK),
   DIN->D2(MOSI),
   GND->GNG,
   3.3V->3.3V

   BLUE -> D2
   VIOLET -> D3
   YELLOW -> D4
   ORANGE -> D5
   GREEN -> D6
   WHITE -> D7
   BLACK -> GND
   RED -> 3.3V

  display.drawPicture(BitmapWaveshare_black, BitmapWaveshare_red, sizeof(BitmapWaveshare_black), sizeof(BitmapWaveshare_red), GxEPD::bm_normal);
  delay(5000);
*/
#include <GxGDEW0154Z04/GxGDEW0154Z04.h>
#include GxEPD_BitmapExamples
#include <GxIO/GxIO_SPI/GxIO_SPI.h>

// fonts
#include <Fonts/DejaVu_Sans_Mono_8.h>
#include <Fonts/DejaVu_Sans_Mono_12.h>
#include <Fonts/Nimbus_Sans_L_Bold_Condensed_16.h>

// icons
#include <imglib/icons_sun.h>
#include <imglib/icons_cloud.h>
#include <imglib/icons_cloud_sun.h>
#include <imglib/icons_rain_1.h>
#include <imglib/icons_rain_2.h>
#include <imglib/icons_rain_3.h>
#include <imglib/icons_snow_1.h>
#include <imglib/icons_snow_2.h>
#include <imglib/icons_snow_3.h>
