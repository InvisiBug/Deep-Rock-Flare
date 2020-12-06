void readIR() {
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.results.value, HEX);

    IrReceiver.resume();  // Receive the next value
  }
}