void startScreen() {
  display.clearDisplay();

  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(F("RGB Torch"));
  display.display();
}

void text(String text) {
  display.clearDisplay();

  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.println(text);
  display.display();
}

void middleText(String text) {
  display.clearDisplay();

  display.setTextSize(2);                        // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);           // Draw white text
  display.setCursor(middle(text.length()), 10);  // Start at top-left corner
  display.println(text);
  display.display();
}

void debugText(String text) {
  display.clearDisplay();

  display.setTextSize(2);                      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);           // Draw white text
  display.setCursor(middle(text.length()), 10);  // Start at top-left corner
  display.println(text);
  display.display();
}

int middle(int textWidth) {
  // Can show a total of 10 characters
  int screenWidth = display.width();
  int charWidth = 12;

  return (screenWidth / 2) - ((charWidth * textWidth) / 2);
}

void border() {
  display.clearDisplay();

  display.drawRect(0, 0, display.width(), display.height(), SSD1306_WHITE);
  display.display();
}