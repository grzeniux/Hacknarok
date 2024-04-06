#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>
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


void setup() 
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  // Compass
  compass.init();
  compass.setCalibration(-1835, 405, -1497, 1178, -2366, 672);

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
}

void loop() {

  measureTemperature();
  GPS();
  CompassThings();
  
}

void delayCustom(unsigned int msDelay){
  unsigned long endTime=millis()+msDelay;
  while(millis()<endTime){
    ;
  }

}
void measureTemperature() {
  // Read analog voltage from thermistor
  int Vo = analogRead(Termistor);
  
  // Calculate thermistor resistance
  float R2 = R1 * (1023.0 / (float)Vo - 1.0);
  
  // Calculate temperature in Celsius
  float logR2 = log(R2);
  float T = 1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2);
  float Tc = T - 273.15;
  
  // Convert temperature to Fahrenheit
  float Tf = (Tc * 9.0) / 5.0 + 32.0;

  // Display temperature
  // Serial.print("Temperatura: ");
  // Serial.print(Tf); // Temperatura w stopniach Farenheita
  // Serial.print(" F; "); // Znak stopni Farenheita
  // Serial.print(Tc); // Temperatura w stopniach Celsjusza
  // Serial.println(" C"); // Znak Stopni Celsjusza 
  
  oled.setTextSize(1);         
  oled.setTextColor(WHITE);    
  oled.setCursor(0, 0);       
  oled.println("Temp: ");                                                           // show_temp
  oled.setCursor(30, 0);        
  oled.print(Tc); 
  oled.println(" C"); 

  oled.println(); 
  oled.display(); 
  oled.clearDisplay();
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
  oled.setCursor(0, 30);       
  oled.print("Lat: ");                                                      // show_lat
  oled.print(gps.location.lat(), 6);
  oled.setCursor(0, 40);       
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
    oled.setCursor(0, 30);       
    oled.println("No valid data"); 
  }
  else
  {
    char dateStr[12];
    sprintf(dateStr, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(dateStr);
    oled.setTextSize(1);         
    oled.setTextColor(WHITE);     
    oled.setCursor(0, 10);       
    oled.print(dateStr);                                                   // show_date
    
    char timeStr[12];
    sprintf(timeStr, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(timeStr);
    oled.setTextSize(1);         
    oled.setTextColor(WHITE);     
    oled.setCursor(0, 20);       
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

//////////////// PRINTOWANIE NA WYSWIETLACZU /////////////////

void displayGPS()
{
  oled.setTextSize(1);         
  oled.setTextColor(WHITE);     
  oled.setCursor(0, 30);       
  oled.println("Latitude: "); 

  oled.setCursor(0, 60);        
  oled.println("Longitude: "); 
  //oled.print(gps.location.lng); 


  oled.println(" C"); //Znak Stopni Celsjusza 

}


////////////////////////////////////////////////////////////