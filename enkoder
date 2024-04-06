volatile int encoder_count = 0;
volatile int last_encoder_count = 0;
volatile int enc;

void encoder_isr() {
    static int8_t prev_AB = 0;
    static int8_t seq[4] = {0, -1, 1, 0};
    int8_t current_AB = (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);
    int8_t encoder_increment = seq[(prev_AB << 2) | current_AB];
    encoder_count += encoder_increment;
    prev_AB = current_AB;
    for(enc=0; enc<100; enc++);
}

void setup() {

    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoder_isr, CHANGE);
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
            // oled.clearDisplay();
            // oled.setCursor(0, 0);
            // oled.println("Option 1 selected");
            // oled.display();
            GPS();
            break;

        case 2:
            oled.clearDisplay();
            oled.setCursor(0, 0);
            oled.println("Option 2 selected");
            oled.display();
            break;
          case 3:
            oled.clearDisplay();
            oled.setCursor(0, 0);
            oled.println("Option 3 selected");
            oled.display();
            break;
        default:
            break;
    }

    delay(100); // Opóźnienie do stabilizacji enkodera
}
