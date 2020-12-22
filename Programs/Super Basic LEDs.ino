#define redPin 7
#define greenPin 6
#define bluePin 5

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  // analogWrite(redPin, 255);
  analogWrite(greenPin, 255);
  // analogWrite(bluePin, 255);
}