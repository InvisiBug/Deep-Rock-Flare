/*
 * IRremote: IRreceiveDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Initially coded 2009 Ken Shirriff http://www.righto.com/
 */

#include <IRremote.h>

int IR_RECEIVE_PIN = 11;

IRrecv IrReceiver(IR_RECEIVE_PIN);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  // Just to know which program is running on my Arduino

  // In case the interrupt driver crashes on setup, give a clue
  // to the user what's going on.
  Serial.println("Enabling IRin");
  IrReceiver.enableIRIn();  // Start the receiver
  // IrReceiver.blink13(true);  // Enable feedback LED

  Serial.print(F("Ready to receive IR signals at pin "));
  Serial.println(IR_RECEIVE_PIN);
}

void loop() {
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.results.value, HEX);

    IrReceiver.resume();  // Receive the next value
  }
  delay(100);
}
