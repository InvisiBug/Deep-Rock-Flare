/*
  +--------+-----------+-------+--------+
  |  Nano  |  MPU6050  |  IR   |  LEDs  |
  +--------+-----------+-------+--------+
  |  D11   |           |  Out  |        |
  |  D6    |           |       |  Red   |
  |  D10   |           |       |  Green |
  |  D5    |           |       |  Blue  |
  |  D2    |  INT      |       |        |
  |  A4    |  SDA      |       |        |
  |  A5    |  SCL      |       |        |
  +--------+-----------+-------+--------+
*/

////////////////////////////////////////////////////////////////////////
//
//  ###
//   #  #    #  ####  #      #    # #####  ######  ####
//   #  ##   # #    # #      #    # #    # #      #
//   #  # #  # #      #      #    # #    # #####   ####
//   #  #  # # #      #      #    # #    # #           #
//   #  #   ## #    # #      #    # #    # #      #    #
//  ### #    #  ####  ######  ####  #####  ######  ####
//
////////////////////////////////////////////////////////////////////////

#include "ColourFade.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Streaming.h"

////////////////////////////////////////////////////////////////////////
//
//  ######
//  #     # ###### ###### # #    # # ##### #  ####  #    #  ####
//  #     # #      #      # ##   # #   #   # #    # ##   # #
//  #     # #####  #####  # # #  # #   #   # #    # # #  #  ####
//  #     # #      #      # #  # # #   #   # #    # #  # #      #
//  #     # #      #      # #   ## #   #   # #    # #   ## #    #
//  ######  ###### #      # #    # #   #   #  ####  #    #  ####
//
////////////////////////////////////////////////////////////////////////
#define redPin 6
#define greenPin 10
#define bluePin 5

#define INTERRUPT_PIN 2

////////////////////////////////////////////////////////////////////////
//
//  #     #
//  #     #   ##   #####  #####  #    #   ##   #####  ######
//  #     #  #  #  #    # #    # #    #  #  #  #    # #
//  ####### #    # #    # #    # #    # #    # #    # #####
//  #     # ###### #####  #    # # ## # ###### #####  #
//  #     # #    # #   #  #    # ##  ## #    # #   #  #
//  #     # #    # #    # #####  #    # #    # #    # ######
//
////////////////////////////////////////////////////////////////////////

// * Effects
ColourFade colourFade(redPin, greenPin, bluePin, 20);

// Gyro
MPU6050 mpu;

////////////////////////////////////////////////////////////////////////
//
//  #     #
//  #     #   ##   #####  #   ##   #####  #      ######  ####
//  #     #  #  #  #    # #  #  #  #    # #      #      #
//  #     # #    # #    # # #    # #####  #      #####   ####
//   #   #  ###### #####  # ###### #    # #      #           #
//    # #   #    # #   #  # #    # #    # #      #      #    #
//     #    #    # #    # # #    # #####  ###### ######  ####
//
////////////////////////////////////////////////////////////////////////
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

// Virtual pin timers
long interval = 250;  // Wait time before taking reading
unsigned long previousMillis = 0;

bool test = false;
bool previousTest = false;

// Debounce stuff
bool lastShakeReading = false;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1;
int buttonState;
int lastButtonState = HIGH;

int mode = 1;
////////////////////////////////////////////////////////////////////////
//
//  ######                                                #####
//  #     # #####   ####   ####  ####¦#    ##   #    #    #     # #####   ##   #####  ##### #    # #####
//  #     # #    # #    # #    # #    #  #  #  ##  ##    #         #    #  #  #    #   #   #    # #    #
//  ######  #    # #    # #      #    # #    # # ## #     #####    #   #    # #    #   #   #    # #    #
//  #       #####  #    # #  ### #####  ###### #    #          #   #   ###### #####    #   #    # #####
//  #       #   #  #    # #    # #   #  #    # #    #    #     #   #   #    # #   #    #   #    # #
//  #       #    #  ####   ####  #    # #    # #    #     #####    #   #    # #    #   #    ####  #
//
////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  setAll(0xffffff);

  startMPU();

  Serial << "| Flare |" << endl;
}

///////////////////////////////////////////////////////////////////////
//
//  #     #                    ######
//  ##   ##   ##   # #    #    #     # #####   ####   ####  #####    ##   #    #
//  # # # #  #  #  # ##   #    #     # #    # #    # #    # #    #  #  #  ##  ##
//  #  #  # #    # # # #  #    ######  #    # #    # #      #    # #    # # ## #
//  #     # ###### # #  # #    #       #####  #    # #  ### #####  ###### #    #
//  #     # #    # # #   ##    #       #   #  #    # #    # #   #  #    # #    #
//  #     # #    # # #    #    #       #    #  ####   ####  #    # #    # #    #
//
///////////////////////////////////////////////////////////////////////
void loop() {
  readAcceleration();
  readYawPitchRoll();
  mode = 1;

  // if (!dmpReady) return;
  if (mode == 1) {
    colourFade.run();
  } else if (mode == 2) {
    setAll(0x00ff00);
  } else if (mode == 3) {
    setAll(0xff7700);
  } else if (mode == 4) {
    setAll(0x00bfff);
  } else if (mode == 5) {
    setAll(0xff1000);
  }

  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
  bool reading = checkShake();
  // Serial << reading << endl;

  // Serial << "Acceleration"
  //        << "\t" << aaReal.x << "\t" << aaReal.y << "\t" << aaReal.z;

  // Serial << "\tYPR"
  //        << "\t" << ypr[0] * 180 / M_PI << "\t" << ypr[1] * 180 / M_PI << "\t" << ypr[2] * 180 / M_PI << endl;

  if (reading != lastButtonState && reading) {
    Serial << "Shake detected" << endl;
    if (mode < 5) {
      mode++;
    } else {
      mode = 1;
    }
    // }
  }

  Serial << mode << endl;
  lastButtonState = reading;

  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;

  //   // if (mode < 5) {
  //   //   mode++;
  //   // } else {
  //   //   mode = 1;
  //   // }
  // }

  // bool reading = checkShake();
  // // Serial << reading << endl;
  // readYawPitchRoll();

  // if (reading != lastButtonState && reading) {
  //   Serial << "Shake detected" << endl;

  //   if ((ypr[2] * 180 / M_PI > 45) && (ypr[2] * 180 / M_PI < 135)) {  // ⬆️
  //     setAll(0x00ff00);
  //   } else if ((ypr[2] * 180 / M_PI > 135) && (ypr[2] * 180 / M_PI < 225)) {  // ➡️
  //     setAll(0xff7700);
  //   } else if ((ypr[2] * 180 / M_PI > 225) && (ypr[2] * 180 / M_PI < 315)) {  // ⬇️
  //     setAll(0x00bfff);
  //   } else {  // ⬅️
  //     setAll(0xff1000);
  //   }
  //   // Serial << "YPR"
  //   //        << "\t" << ypr[0] * 180 / M_PI << "\t" << ypr[1] * 180 / M_PI << "\t" << ypr[2] * 180 / M_PI << endl;
  // }

  // lastButtonState = reading;

  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;
  //   readYawPitchRoll();
  // }
  // Serial << "YPR"
  //        << "\t" << ypr[0] * 180 / M_PI << "\t" << ypr[1] * 180 / M_PI << "\t" << ypr[2] * 180 / M_PI << endl;

  // if ((millis() - lastDebounceTime) > debounceDelay) {
  //   // whatever the reading is at, it's been there for longer than the debounce
  //   // delay, so take it as the actual current state:

  //   // if the button state has changed:
  //   if (reading != buttonState) {
  //     buttonState = reading;

  //     // only toggle the LED if the new button state is HIGH
  //     if (buttonState == HIGH) {
  //       Serial << "Shake" << endl;
  //     }
  //   }
  // }

  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

  //   readAcceleration();
  //   // Serial << "Acceleration"
  //   //        << "\t" << aaReal.x << "\t" << aaReal.y << "\t" << aaReal.z;

  //   int threshold = 8000;

  //   if ((aaReal.x > threshold || aaReal.x < -threshold) ||
  //       (aaReal.y > threshold || aaReal.y < -threshold) ||
  //       (aaReal.z > threshold || aaReal.z < -threshold)) {
  //     // if (!test) {
  //     //   test = true;
  //     //   Serial << "Shake" << endl;
  //     // } else {
  //     //   test = false;
  //     // }
  //     // digitalWrite(virtualPin, HIGH);
  //   }
  // }

  // Serial << "Virtual Pin:\t" << digitalRead(virtualPin) << endl;

  // setAll(0x00bfff);  // Scout
  // delay(5000);

  // setAll(0xff1000);  // Engie
  // delay(5000);

  // setAll(0xff7700);  // Driller
  // delay(5000);

  // setAll(0x00ff00);  // Gunner
  // delay(5000);

  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;
  //   choice = colours[random(0, 25)];
  // }
}

void setAll(unsigned long colour) {
  int red = (colour & 0xff0000) >> 16;
  int green = (colour & 0x00ff00) >> 8;
  int blue = (colour & 0x0000ff);

  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

void dmpDataReady() {
  mpuInterrupt = true;
}