#include "ColourFade.h"

#define redPin 6
#define greenPin 10
#define bluePin 5

ColourFade colourFade(redPin, greenPin, bluePin, 20);

// uint32_t choice;

// long interval = (5 * 1000);  // Wait time before taking reading
// unsigned long previousMillis = 0;

// uint32_t colours[24] = {  // https://www.w3schools.com/colors/colors_picker.asp
//     0xff4000, 0xff8000, 0xffbf00, 0xffff00, 0xbfff00, 0x80ff00,
//     0x40ff00, 0x00ff00, 0x00ff40, 0x00ff80, 0x00ffbf, 0x00ffff,
//     0x00bfff, 0x0080ff, 0x0040ff, 0x0000ff, 0x4000ff, 0x8000ff,
//     0xbf00ff, 0xff00ff, 0xff00bf, 0xff0080, 0xff0040, 0xff0000};

// // newRed = (newColour & 0xff0000) >> 16;
// // newGreen = (newColour & 0x00ff00) >> 8;
// // newBlue = (newColour & 0x0000ff);

void setup() {
  // choice = colours[20];

  // pinMode(redPin, OUTPUT);
  // pinMode(greenPin, OUTPUT);
  // pinMode(bluePin, OUTPUT);
}

void loop() {
  colourFade.run();
  // // analogWrite(redPin, 15);
  // // analogWrite(greenPin, 50);
  // // analogWrite(bluePin, 255);
  // // serial.println(choice);

  // analogWrite(redPin, (choice & 0xff0000) >> 16);
  // analogWrite(greenPin, (choice & 0x00ff00) >> 8);
  // analogWrite(bluePin, (choice & 0x0000ff));

  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;
  //   choice = colours[random(0, 25)];
  // }
}