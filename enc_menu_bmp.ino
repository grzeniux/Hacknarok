#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_BMP280.h>


//OLED
#define i2c_Address 0x3c
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

//ENC
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

volatile int encoder_count = 0;
volatile int last_encoder_count = 0;

//bmp
Adafruit_BMP280 bmp;

void encoder_isr() {
    static int8_t prev_AB = 0;
    static int8_t seq[4] = {0, -1, 1, 0};
    int8_t current_AB = (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);
    int8_t encoder_increment = seq[(prev_AB << 2) | current_AB];
    encoder_count += encoder_increment;
    prev_AB = current_AB;

}

void setup() {
    // if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //     Serial.println(F("SSD1306 allocation failed"));
    //     for(;;);
    // }
  display.begin(i2c_Address, true); 
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoder_isr, CHANGE);

      unsigned status;
      status = bmp.begin(0x76);
    display.display();
    delay(2000); // Zaczekaj 2 sekundy przed rozpoczęciem
    display.clearDisplay();

    display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("Encoder Menu:");
    display.display();

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    last_encoder_count = encoder_count;
}

void loop() {
    // if (encoder_count != last_encoder_count) {
    //     last_encoder_count = encoder_count;

    //     display.clearDisplay();
    //     display.setCursor(0, 0);
    //     display.print("Encoder Count: ");
    //     display.println(encoder_count);
    //     display.display();
    // }

    // Obsługa enkodera na żywo - do zmiany wyświetlanej opcji menu
    int menu_option = encoder_count % 3 + 1;

    switch (menu_option) {
        case 1:
            display.clearDisplay();
            display.setCursor(0, 0);
            // display.println("Option 1 selected");
            display.print("Temp = ");
            display.print(bmp.readTemperature());
            display.print("Pres = ");
            display.print(bmp.readPressure());
            display.print("Approx altitude = ");
            display.print(bmp.readTemperature());
            display.display();
            break;
        case 2:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Option 2 selected");
            display.display();
            break;
          case 3:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Option 3 selected");
            display.display();
            break;
        default:
            break;
    }

    delay(100); // Opóźnienie do stabilizacji enkodera
}
