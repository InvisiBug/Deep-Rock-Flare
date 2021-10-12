void diy() {
  redPercentage = map(analogRead(redPot), potMin, potMax, 0, 100);
  greenPercentage = map(analogRead(greenPot), potMin, potMax, 0, 100);
  bluePercentage = map(analogRead(bluePot), potMin, potMax, 0, 100);

  analogWrite(redPin, map(redPercentage, 0, 100, 0, 255));
  analogWrite(greenPin, map(greenPercentage, 0, 100, 0, 255));
  analogWrite(bluePin, map(bluePercentage, 0, 100, 0, 255));
}

void debug() {
  // middleText("1234567890");
  // middleText(String(analogRead(redPot) + "," + analogRead(greenPot) + analogRead(bluePot)));

  String string = String(analogRead(redPot));
  string.concat(" ");
  string.concat(analogRead(greenPot));
  string.concat(" ");
  string.concat(analogRead(bluePot));
  debugText(string);
}

void off() {
  middleText(F("Off"));

  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);
}