#if defined(ARDUINO)
  SYSTEM_MODE(MANUAL);
#endif

// This #include statement was automatically added by the Particle IDE.
#include "PietteTech_DHT.h"

// NTP time
//#include <SparkTime.h>

// MQ-135 (GAS)
#include "mq135.h"

// BMP-180 (pressure/temp)
#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"

// BH-1750 (light)
#include "BH1750.h"

// DHT22 (humidity/temp)
#include "PietteTech_DHT.h"

// e-Ink display
#include <GxEPD.h>
#include <GxGDEW0154Z04/GxGDEW0154Z04.cpp>
#include GxEPD_BitmapExamples
#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>
#include "unicorn_helmet_ink.h"

// HC-SR04 (ultasonic)
#include "hc_sr04.h"

#define DHTTYPE  DHT22             // Sensor type DHT11/21/22/AM2301/AM2302
#define DHTPIN   D15               // Digital pin for DHT22
#define PIN_MQ135 A6               // Anlog pin for GAS sensor
#define PIN_TRCT A0                // Anlog pin for IR sensor
#define HAS_RED_COLOR
#define IR_PROBE_DELAY 5
//declarations
void checkEnvironment();
int get_snow(int n = 40);

void dht_wrapper(); // must be declared before the lib initialization

static const uint8_t DC    = D6;
static const uint8_t RST   = D7;

BMP280 bmp;
MQ135 mq = MQ135(PIN_MQ135);
UDP UDPClient;
//SparkTime rtc;
BH1750 bh;
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);
// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t b
GxIO_Class io(SPI1, D5, DC);
GxEPD_Class display(io, RST, D3);
Ultrasonic ultrasonic(A1, 12);

int led2 = D7;
bool bmp_online = false;
bool mq_online = false;
bool bh_online = false;
bool dht_online = false;
bool ink_online = false;
unsigned long currentTime;
unsigned long lastTime = 0UL;
bool bDHTstarted;

//Timer timer(30000, checkEnvironment);

void setup(){
    //Particle.connect();
    Serial.begin(115200);
    Wire.begin();
    display.init();

    rtc.begin(&UDPClient, "north-america.pool.ntp.org");
    rtc.setTimeZone(1); // gmt offset

    if (bmp.initialize()) {
        Particle.publish("DEBUG", "BMP280 sensor found...");
        bmp_online = true;
    } else {
        Particle.publish("WARN", "Could not find a valid BMP280 sensor, check wiring!");
    }
    if (mq.getRZero() > 1) {
        Particle.publish(String(mq.getRZero()), "MQ135 sensor found...");
        mq_online = true;
    } else {
        Particle.publish("WARN", "Could not find a valid MQ135 sensor, check wiring!");
    }
    if (bh.begin(BH1750::ONE_TIME_HIGH_RES_MODE)){
        Particle.publish("DEBUG", "BH1750 sensor found...");
        bh_online = true;
    }
    Particle.publish(String(analogRead(DHTPIN)), "DHT22 sensor found...");
    if (get_dht_status()){
        Particle.publish(String(analogRead(DHTPIN)), "DHT22 sensor found...");
        dht_online = true;
    }

    //timer.start();
    Particle.publish("DEBUG", "started!");
    pinMode(PIN_TRCT, INPUT);
}

void loop(){

    delay(30000);
    connectWiFi();
    //int ad_value;
    //ad_value = analogRead(A0);
    //Particle.publish("ad_value", String(ad_value));

    //float temp = 22.0;
    //String results = ultrasonic.get_n_readings_pct(temp, 10);
    //Serial.println(results);
    //display.fillRect(0, 0, 200, 200, GxEPD_WHITE);
    //display.setTextColor(GxEPD_BLACK);
    //display.setCursor(0, 0);
    //display.println(results);
    //display.update();
    //Particle.publish("results", results);
    //Serial.println();
    checkEnvironment();
    WiFi.off();


}

void connectWiFi() {
    String known[3][2] = {
      { "LittleDragon", "00000000"},
      { "DARKOPL_2", "00000000" },
      { "SMOKI_SOKOLEC", "00000000" }
    };
    WiFi.on();
    WiFiAccessPoint aps[20];
    int found = WiFi.scan(aps, 20);
    if (found > 0) {
        for (int i = 0; i < found; i++) {
            Serial.println(aps[i].ssid);
            for (int j = 0; j < 3; j++) {
                if(String(aps[i].ssid) == known[j][0]) {
                      Serial.println("MATCH!");
                      WiFi.setCredentials(known[j][0].c_str(),known[j][1].c_str());
                      WiFi.connect();

                      while ( WiFi.connecting()) {
                        // print dots while we wait to connect
                        Serial.print(".");
                        delay(300);
                      }

                      Serial.println("\nYou're connected to the network");
                      Serial.println("Waiting for an ip address");
                      break;
                }
            }
        }
    }
    Particle.connect();
    Particle.process();

}

void checkEnvironment() {
    //Particle.connect();
    // update time
    //currentTime = rtc.now();
    //Particle.publish("Measurment time", rtc.ISODateString(currentTime));

    // pressure + temp
    float temp, pressure, alt;
    if (bmp_online) {
        get_pressure(temp, pressure, alt);
        Particle.publish("environment/temperature", String(temp));
        Particle.publish("environment/pressure", String(pressure));
        Particle.publish("environment/altitude", String(alt));
        delay(1000);
    } else {
        temp = 21.0;
    }
    float dht_temp, humidity, dew;
    bool dht_done = false;
    if (bmp_online) {
        get_humidity(dht_temp, humidity, dew, dht_done);
        Particle.publish("dht22/temperature", String(dht_temp));
        Particle.publish("dht22/humidity", String(humidity));
        Particle.publish("dht22/dew", String(dew));
        delay(1000);
    }
    if (dht_done) {
      temp = (temp + dht_temp) / 2;
    } else {
      humidity = 25.0;
    }
    // GAS
    float ppm;
    if (mq_online) {
        ppm = mq.getCorrectedPPM(temp, humidity);
        Particle.publish("environment/CO2", String(ppm));
        // get_gas(temp, humidity, ppm);  //for calibration output
    }
    // light
    uint16_t luxvalue;
    if (bh_online) {
        luxvalue = bh.readLightLevel();
         Particle.publish("environment/light", String(luxvalue));
    }

    // check for rain
    unsigned int rain_intensity = ultrasonic.get_n_readings_pct(temp, 40);
    Particle.publish("environment/rain_intensity", String(rain_intensity));

    // check for snow
    unsigned int snow_intensity = 0;
    if (temp <= 10) {
        snow_intensity = get_snow();
    }
    Particle.publish("environment/snow", String(snow_intensity));

    // show results
    updateInk(temp, humidity, pressure, ppm,  luxvalue, rain_intensity, snow_intensity);

    //if (Particle.connected()) {
    //    Particle.process();
    //    Particle.disconnect();
    //}

}

void get_pressure(float& temperature, float& pascal, float& meters) {
    bmp.setEnabled(0);
    bmp.triggerMeasurement();
    bmp.getTemperature(temperature);
    bmp.getPressure(pascal);
    bmp.getAltitude(meters);
    pascal = pascal / 100;
}

void get_gas(float& temperature, float& humidity, float& corrected_ppm) {
    // assume current humidity. Recommended to measure with DHT22
    float rzero = mq.getRZero();
    float correctedRZero = mq.getCorrectedRZero(temperature, humidity);
    float resistance = mq.getResistance();
    float ppm = mq.getPPM();
    corrected_ppm = mq.getCorrectedPPM(temperature, humidity);
    Particle.publish("environment/rzero", String(rzero));
    Particle.publish("environment/correctedRZero", String(correctedRZero));
    Particle.publish("environment/resistance", String(resistance));
}

void dht_wrapper() {
    DHT.isrCallback();
}

bool get_dht_status() {
    DHT.acquire();
    int result = DHT.getStatus();
    Particle.publish(String(result));
    switch (result) {
    case DHTLIB_OK:
        return true;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        //Serial.println("Error\n\r\tChecksum error");
        break;
    case DHTLIB_ERROR_ISR_TIMEOUT:
        //Serial.println("Error\n\r\tISR time out error");
        break;
    case DHTLIB_ERROR_RESPONSE_TIMEOUT:
        //Serial.println("Error\n\r\tResponse time out error");
        break;
    case DHTLIB_ERROR_DATA_TIMEOUT:
        //Serial.println("Error\n\r\tData time out error");
        break;
    case DHTLIB_ERROR_ACQUIRING:
        return true;
        break;
    case DHTLIB_ERROR_DELTA:
        //Serial.println("Error\n\r\tDelta time to small");
        break;
    case DHTLIB_ERROR_NOTSTARTED:
        //Serial.println("Error\n\r\tNot started");
        break;
    default:
        //Serial.println("Unknown error");
        break;
    }
    return false;
}

void get_humidity(float& temperature, float& humidity, float& dew, bool& performed) {
    if (!bDHTstarted) {
        DHT.acquire();
        bDHTstarted = true;
    }
    if (!DHT.acquiring()) {
        int result = DHT.getStatus();
        switch (result) {
        case DHTLIB_OK:
            Particle.publish("OK");
            performed = true;
            break;
        case DHTLIB_ERROR_CHECKSUM:
            delay(1000);
            Particle.publish("Error\n\r\tChecksum error");
            break;
        case DHTLIB_ERROR_ISR_TIMEOUT:
            delay(1000);
            Particle.publish("Error\n\r\tISR time out error");
            break;
        case DHTLIB_ERROR_RESPONSE_TIMEOUT:
            delay(1000);
            Particle.publish("Error\n\r\tResponse time out error");
            break;
        case DHTLIB_ERROR_DATA_TIMEOUT:
            delay(1000);
            Particle.publish("Error\n\r\tData time out error");
            break;
        case DHTLIB_ERROR_ACQUIRING:
            delay(1000);
            Particle.publish("Error\n\r\tAcquiring");
            break;
        case DHTLIB_ERROR_DELTA:
            delay(1000);
            Particle.publish("Error\n\r\tDelta time to small");
            break;
        case DHTLIB_ERROR_NOTSTARTED:
            delay(1000);
            Particle.publish("Error\n\r\tNot started");
            break;
        default:
            delay(1000);
            Particle.publish("Unknown error");
            break;
        }

        humidity = DHT.getHumidity();
        temperature = DHT.getCelsius();
        dew = DHT.getDewPoint();
        bDHTstarted = false;
    }
}

int get_snow(int n) {
    int successes = 0;
    for (int i=0; i<n; i++) {
        if (analogRead(PIN_TRCT) < 3990) {
            successes++;
        }
        delay(IR_PROBE_DELAY);
    }
    return successes ? (successes / n * 100) : successes;
}

const unsigned char *get_cloud_icon(int rain_intensity, int snow_intensity){
    if ((snow_intensity > 0) && (snow_intensity <= 15)) {
        return icons_snow_1;
    } else if ((snow_intensity > 15) && (snow_intensity <= 60)) {
        return icons_snow_2;
    } else if (snow_intensity > 60){
        return icons_snow_3;
    }
    if (rain_intensity==0 ) {
        return icons_cloud;
    } else if (rain_intensity<=15) {
        return icons_rain_1;
    } else if (rain_intensity<=60) {
        return icons_rain_2;
    } else {
        return icons_rain_3;
    }
}


void updateInk(float& temp, float& humidity, float& pressure, float& ppm, uint16_t& light, unsigned int& rain, unsigned int& snow) {
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
    */

    //display.drawPicture(BitmapWaveshare_black, BitmapWaveshare_red, sizeof(BitmapWaveshare_black), sizeof(BitmapWaveshare_red), GxEPD::bm_normal);
    //delay(5000);




    const unsigned char *cloud_icon = get_cloud_icon(rain, snow);
    //const unsigned char *sun_icon = get_sun_icon(light);

    const GFXfont* f8 = &DejaVu_Sans_Mono_8;
    const GFXfont* f12 = &DejaVu_Sans_Mono_12;
    const GFXfont* f16 = &Nimbus_Sans_L_Bold_Condensed_16;

    display.setRotation(3);
    display.fillRect(0, 0, 200, 200, GxEPD_WHITE);
    // First row (status | date)
    display.drawRect(0, 0, 100, 20, GxEPD_BLACK);
    display.fillRect(100, 0, 100, 20, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(f8);
    display.setCursor(110, 14);
    display.println("18-01-06 15:55");

    // 2nd row (TMP | HUM | Pressure)
    display.fillRect(0, 19, 200, 24, GxEPD_RED);
    display.drawRect(0, 19, 50, 60, GxEPD_BLACK);
    display.drawRect(49, 19, 51, 60, GxEPD_BLACK);
    display.drawRect(99, 19, 100, 60, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(f12);
    display.setCursor(15, 35);
    display.println("TMP");
    display.setCursor(65, 35);
    display.println("HUM");
    display.setCursor(115, 35);
    display.println("PRESSURE");
    display.setTextColor(GxEPD_BLACK);
    display.setFont(f16);
    display.setCursor(5, 65);
    display.print(temp, 1);
    display.print("C");
    display.setCursor(55, 65);
    display.print(humidity, 1);
    display.print("%");
    display.setCursor(109, 65);
    display.print(pressure, 1);
    display.print(" HPa");

    // 4th row (RAIN / CLOUDS)

    display.drawRect(0, 80, 100, 59, GxEPD_BLACK);
    display.drawRect(99, 80, 100, 59, GxEPD_BLACK);
    display.drawBitmap(icons_sun, 24, 85, 50, 50, GxEPD_WHITE);
    display.drawBitmap(cloud_icon, 124, 85, 50, 50, GxEPD_WHITE);

    // 5th row (CO2 / DEW POINT)
    display.fillRect(0, 140, 200, 24, GxEPD_RED);
    display.drawRect(0, 140, 100, 60, GxEPD_BLACK);
    display.drawRect(99, 140, 100, 60, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(f12);
    display.setCursor(15, 157);
    display.println("AIR C02");
    display.setCursor(115, 157);
    display.println("Light");
    display.setTextColor(GxEPD_BLACK);
    display.setFont(f16);
    display.setCursor(12, 187);
    display.print(ppm, 1);
    display.print(" ppm");
    display.setCursor(135, 187);
    display.print(light);
    display.print(" LUX");

    display.update();
}
