#include <WiFi.h>
#include <PubSubClient.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/adc.h"

#include <GyverOLED.h>    // GPS
#include <charMap.h>      // GPS
#include <icons_7x7.h>    // GPS
#include <icons_8x8.h>    // GPS

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


#include <QMC5883LCompass.h> // Compass

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define ADC_PIN 26

//Enc
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

#define MAX_VILLAGE_NAME_LENGTH 15
#define DEGREES_TO_METERS_LATITUDE 78710 
#define DEGREES_TO_METERS_LONGITUDE 78605 

Adafruit_BMP280 bmp;

// const char* ssid = "Essa";
// const char* password = "alejaja123";

const char* ssid = "KPT-Conference";
const char* password = "E2ue6Tm&";

// MQTT broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic_publish = "botfreaks/mqttfxtest/data";
const char* mqtt_topic_subscribe = "botfreaks/mqttfxtest/commands";

WiFiClient espClient;
PubSubClient client(espClient);


volatile int encoder_count = 0;
volatile int last_encoder_count = 0;

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//program odczytujacy temperature z termistora
int Termistor = 26; //przypisanie pinu odczytywania napiecia z termistora
int Vo; //odczyt napiecia dla termistora
float R1 = 10000; //Rezystor staly 10kiloomow
float logR2, R2, T, Tc, Tf; //deklaracja danych wejsciowych i wyjsciowych
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

static const int RXPin = 1, TXPin = 0;    // GPS
static const uint32_t GPSBaud = 9600;     // GPS
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


// Compass
// Mode Control (MODE)
const byte qmc5883l_mode_stby = 0x00;
const byte qmc5883l_mode_cont = 0x01;
// Output Data Rate (ODR)
const byte qmc5883l_odr_10hz  = 0x00;
const byte qmc5883l_odr_50hz  = 0x04;
const byte qmc5883l_odr_100hz = 0x08;
const byte qmc5883l_odr_200hz = 0x0C;
// Full Scale Range (RNG)
const byte qmc5883l_rng_2g    = 0x00;
const byte qmc5883l_rng_8g    = 0x10;
// Over Sample Ratio (OSR)
const byte qmc5883l_osr_512   = 0x00;
const byte qmc5883l_osr_256   = 0x40;
const byte qmc5883l_osr_128   = 0x80;
const byte qmc5883l_osr_64    = 0xC0;

QMC5883LCompass compass;

float TC;

const char *villages[12]  = {"Klakegg",
    "Refvik",
    "Skei",
    "Skelige",
    "Utvik",
    "Vassenden",
    "Bjornheim",
    "Jotunheim",
    "Yggdrasil",
    "Midgardr",
    "Valhalla",
    "Njordvik"
};

struct Localisation {
    float longitude;
    float latitude;
    char villageName[MAX_VILLAGE_NAME_LENGTH];
};

struct Journey {
    float duration;
    float distance;
    char direction[3];
};


struct CompassData{
  int x_value;
  int y_value;
  int azimuth;
  char direction[4];
};

double lat = 0, lng = 0;
TinyGPSDate gpsDate;
TinyGPSTime gpsTime;
Localisation local;
Localisation pineska = {50.0674, 19.9128, "Miasteczko"};
    double minLatitude = 49.955, maxLatitude = 50.05;
    double minLongitude = 19.81, maxLongitude = 20.23;
    int randValue = 0+rand()%(11-0+1);
  Journey road;

    
void setup() 
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  // Compass
  compass.init();
  compass.setCalibration(-1835, 405, -1497, 1178, -2366, 672);

  //ENC
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  unsigned status;
  status = bmp.begin(0x76);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoder_isr, CHANGE);
  // initialize OLED display with address 0x3C for 128x64
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }



  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  // oled.println("Hello World!"); // text to display
  oled.display();               // show on OLED

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  last_encoder_count = encoder_count;



     ///// // Connect to Wi-Fi   ///////////////////////
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Connect to MQTT broker
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32_C6")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
    srand((unsigned int)time(NULL));
    local.latitude=generate_random(minLatitude, maxLatitude);
    local.longitude=generate_random(minLongitude, maxLongitude);
    strcpy(local.villageName, villages[randValue]);
  
}

void loop() {
  CompassData compassData = readCompass();
    static int last_menu_option = 0;
   int menu_option = encoder_count % 3 + 1;
    measureTemperature(TC);
    //updateGPSData();
    calc_journey(&road,pineska,local);
    if (menu_option != last_menu_option) {
      switch (menu_option) {
          case 1:
              
                oled.setTextSize(1);
                oled.setTextColor(WHITE);
                oled.clearDisplay();
                oled.setCursor(0,0);
                oled.print("Comp:");
                oled.setCursor(70,0);
                oled.print("Dir: ");
                displayCompassData(compassData);
                oled.setCursor(70,10);
                oled.println(road.direction);
                oled.setCursor(0,20);
                oled.print("Village: ");
                oled.println(local.villageName);
                // oled.setCursor(0,20);
                // oled.print("Lat: ");
                // oled.println(local.latitude);
                // oled.setCursor(0,30);
                // oled.print("Long: ");
                // oled.println(local.longitude);
                oled.setCursor(0,30);
                oled.print("Time: ");
                oled.println(road.duration);
                oled.setCursor(0,40);
                oled.print("Distance: ");
                oled.println(road.distance);
                oled.display();
              break;
          case 2:
              oled.setTextSize(1);
              oled.setTextColor(WHITE);
              oled.clearDisplay();
              oled.setCursor(0,0);
              oled.print("temp: ");
              oled.print(TC);
              oled.print(" C");
              oled.display();
              break;
            case 3:
            while (menu_option == 3) {
                    // Kontynuuj wywoływanie GPS() do momentu zmiany menu_option
                    GPS(); // Zakładając, że funkcja ta wywołuje GPS() i aktualizuje wyświetlacz

                    // Opcjonalnie: Możesz dodać niewielkie opóźnienie, aby nie zablokować całkowicie pętli
                    delay(10);

                    // Sprawdź, czy menu_option zmieniło się podczas wykonania pętli
                    int current_encoder_position = menu_option; // Załóżmy, że masz funkcję do odczytu pozycji enkodera
                    menu_option = current_encoder_position % 3 + 1;
                }

              oled.display();
              break;
          default:
              break;
      }
      delay(100);
    }

  ///////////////////// MQTT ///////////
  client.loop();

  // Publish sample data every 5 seconds
  static unsigned long lastMsg = 0;
if (millis() - lastMsg > 5000) 
{
  lastMsg = millis();


  String black_string = String(20);
  String red_string = String(40);

  client.publish("black", black_string.c_str());
  client.publish("red", red_string.c_str());


  // char tempString[8];
  // dtostrf(measureTemperature(TC), 2, 2, tempString);
  // client.publish("temperature", tempString);

  // float measureTemp = measureTemperature();
  float pressureValue = bmp.readPressure();
  float Temperature_BMP = bmp.readTemperature();

  String pressureString = String(pressureValue, 3);
  String temperatureString_BMP = String(Temperature_BMP, 2);
  // String measureTemperature = String(measureTemp,2)

  client.publish("Pressure", pressureString.c_str()); 
  client.publish("Temperature_BMP", temperatureString_BMP.c_str());
  // client.publish("Measure_Temp", measureTemperature.c_str());



  static unsigned long lastGpsMsg = 0;

  String latitudeString = String(gps.location.lat(), 6);
  String longitudeString = String(gps.location.lng(), 6);

  client.publish("Latitude", latitudeString.c_str()); 
  client.publish("Longitude", longitudeString.c_str());

///////////////////////// NTC //////////////
  measureTemperature(TC);
  float temperature = TC;
  String temperatureString = String(temperature, 2);
  client.publish("Temperature", temperatureString.c_str());
  
////////////////// COMPASS ///////////////
  String compassDataString = String(compassData.azimuth) + "," + 
                             String(compassData.direction);

  client.publish("CompassData", compassDataString.c_str());


 // Odczytaj czas z GPS
String timeStr;
if (gps.time.isValid()) {
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Formatuj godzinę, minutę i sekundę jako dwucyfrowe
    timeStr = String(hour < 10 ? "0" + String(hour) : String(hour)) + ":" +
              String(minute < 10 ? "0" + String(minute) : String(minute)) + ":" +
              String(second < 10 ? "0" + String(second) : String(second));
} else {
    timeStr = "Invalid";
}

// Wysłanie danych do MQTT
client.publish("GPS/Time", timeStr.c_str());





// Odczytaj datę z GPS
String dateStr;
if (gps.date.isValid()) {
    dateStr = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
} else {
    dateStr = "Invalid";
}

// Wysłanie danych do MQTT
client.publish("GPS/Date", dateStr.c_str());



//////////// GPS ///////////

float courseValue = gps.course.deg();
bool courseValid = gps.course.isValid();
client.publish("GPS/Course", String(courseValue, 2).c_str());

int satellitesValue = gps.satellites.value();
bool satellitesValid = gps.satellites.isValid();
client.publish("GPS/Satellites", String(satellitesValue).c_str());


  } /// END IF MILLIS()

}

void delayCustom(unsigned int msDelay){
  unsigned long endTime=millis()+msDelay;
  while(millis()<endTime){
    ;
  }

}

void encoder_isr() {
    static int8_t prev_AB = 0;
    static int8_t seq[4] = {0, -1, 1, 0};
    int8_t current_AB = (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);
    int8_t encoder_increment = seq[(prev_AB << 2) | current_AB];
    encoder_count += encoder_increment;
    prev_AB = current_AB;

}

void measureTemperature(float &TC) {
  // Read analog voltage from thermistor
  int Vo = analogRead(Termistor);
  
  // Calculate thermistor resistance
  float R2 = R1 * (1023.0 / (float)Vo - 1.0);
  
  // Calculate temperature in Celsius
  float logR2 = log(R2);
  float T = 1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2);
  TC = T - 273.15;
  
  // Convert temperature to Fahrenheit
  float Tf = (Tc * 9.0) / 5.0 + 32.0;

  // Display temperature
  // Serial.print("Temperatura: ");
  // Serial.print(Tf); // Temperatura w stopniach Farenheita
  // Serial.print(" F; "); // Znak stopni Farenheita
  // Serial.print(Tc); // Temperatura w stopniach Celsjusza
  // Serial.println(" C"); // Znak Stopni Celsjusza 
  delayCustom(500);
}




void GPS()
{
  static const double wioska_wikingow = 50.0682265, LONDON_LON = 19.905683059764645;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      wioska_wikingow, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      wioska_wikingow, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(500);
  //delayCustom(500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  // Wyświetlanie szerokości i długości geograficznej na ekranie OLED
  oled.setTextSize(1);         
  oled.setTextColor(WHITE);  
  oled.setCursor(0, 20);       
  oled.print("Lat: ");                                                      // show_lat
  oled.print(gps.location.lat(), 6);
  oled.setCursor(0, 30);       
  oled.print("Lng: ");                                                      // show_lng
  oled.print(gps.location.lng(), 6);
}

void CompassThings(){
  int x_value;
  int y_value;
  int azimuth;
  char direction[strlen("NNE") + 1];
  compass.read(); // Read compass values via I2C
  x_value = compass.getX();
  y_value = compass.getY();
  azimuth = compass.getAzimuth();
  compass.getDirection(direction,azimuth);
  direction[3]='\0';

  oled.setTextSize(1);         
  oled.setTextColor(WHITE);     
  oled.setCursor(0, 50);       
  oled.print(direction);
}

CompassData readCompass(){
  CompassData data;
  compass.read();
  data.x_value=compass.getX();
  data.y_value=compass.getY();
  data.azimuth = compass.getAzimuth();
  compass.getDirection(data.direction, data.azimuth);
  data.direction[3]='\0';

  return data;
}

void displayCompassData(const CompassData& data){
  oled.setCursor(0,10);
  oled.print(data.direction);
  //oled.display();
}
///////////////// FUNKCJE DO GPS'A /////////////
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid() || !t.isValid())
  {
    Serial.println(F("********** "));
    oled.setTextSize(1);         
    oled.setTextColor(WHITE);
    oled.clearDisplay();     
    oled.setCursor(0, 0);       
    oled.println("No valid data"); 
  }
  else
  {
    char dateStr[12];
    sprintf(dateStr, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(dateStr);
    oled.setTextSize(1);         
    oled.setTextColor(WHITE); 
    oled.clearDisplay();    
    oled.setCursor(0,0);       
    oled.print(dateStr);                                                   // show_date
    
    char timeStr[12];
    sprintf(timeStr, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(timeStr);
    oled.setTextSize(1);         
    oled.setTextColor(WHITE);     
    oled.setCursor(0, 10);       
    oled.print(timeStr);                                                  // show_time
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

void calc_journey(Journey *road, Localisation gps, Localisation destination){
  
  float ref_latitude = destination.latitude - gps.latitude;
  float ref_longitude = destination.longitude - gps.longitude;
  road->distance = hypot(ref_latitude*DEGREES_TO_METERS_LATITUDE, ref_longitude*DEGREES_TO_METERS_LATITUDE)/1000;
  road->duration = road->distance/5.0;



  if (ref_latitude == 0) {
    // Handle cases when ref_latitude is 0
    if (ref_longitude > 0) road->direction[0] = 'E';
    else if (ref_longitude < 0) road->direction[0] = 'W';
  } 
  else if (ref_longitude == 0) {
    // Handle cases when ref_latitude is 0
    if (ref_latitude > 0) road->direction[0] = 'N';
    else if (ref_latitude < 0) road->direction[0] = 'S';
  } 
  else if (ref_latitude > 0){
    (ref_longitude > 0) ? strcpy(road->direction, "NE") : strcpy(road->direction, "NW");
  }
  
  else{
    (ref_longitude > 0) ? strcpy(road->direction, "SE") : strcpy(road->direction, "SW");
  }
}

double generate_random(double min, double max){
    return min+(double)rand()/((double)RAND_MAX / (max-min));
}
