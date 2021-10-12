void button1Clicked() {
  if (mode < 4) {
    mode++;
  } else {
    mode = 0;
  }
}

void button1Held() {
  Serial << "Button One Held" << endl;
}

void button2Clicked() {
  Serial << "Button One Clicked" << endl;
}

void button2Held() {
  Serial << "Button One Held" << endl;
}