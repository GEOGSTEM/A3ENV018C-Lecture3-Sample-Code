#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 SSD1306(128, 64);

void setup() {
  SSD1306.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  SSD1306.setRotation(2);
  SSD1306.setTextSize(1);
  SSD1306.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  SSD1306.invertDisplay(false);
  SSD1306.clearDisplay();
  SSD1306.display();
}

unsigned int y = 0;
float T_values[] = {25.4, 25.6, 25.9};

void loop() {
  SSD1306.clearDisplay();
  SSD1306.setCursor(0, 0);
  SSD1306.println("T: " + String(T_values[y]));
  SSD1306.display();

  // Delay for 5 seconds
  delay(5000);

  // Increment y and loop back to 0 if y reaches the end
  y = (y + 1) % 3;
}
