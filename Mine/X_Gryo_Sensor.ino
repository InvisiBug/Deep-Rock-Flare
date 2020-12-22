void startSensor() {
  Serial << "Initializing Gyro..." << endl;
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection

  Serial << "Checking Connection" << endl;
  Serial.println(F("Testing device connections..."));
  // mpu.testtestConnection() ? Serial << "MPU6050 connection successful" << endl : Serial << "MPU6050 connection failed" << endl;
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read())
  //   ;  // empty buffer
  // while (!Serial.available())
  //   ;  // wait for data
  // while (Serial.available() && Serial.read())
  //   ;  // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu.setXGyroOffset(72);
  // mpu.setYGyroOffset(-51);
  // mpu.setZGyroOffset(28);
  // mpu.setXAccelOffset(-4254);
  // mpu.setYAccelOffset(-1984);
  // mpu.setZAccelOffset(1442);  // 1688 factory default for my test chip

  mpu.setXAccelOffset(-4284);
  mpu.setYAccelOffset(-1938);
  mpu.setZAccelOffset(1465);
  mpu.setXGyroOffset(71);
  mpu.setYGyroOffset(-47);
  mpu.setZGyroOffset(32);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // turn on the DMP, now that it's ready
    // enable Arduino interrupt detection

    Serial << "Enabling interrupt detection (Arduino external interrupt" << endl;
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    Serial << "DMP Initialization failed, code:" << devStatus << endl;
  }
}

void readGyro() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    // yawPitchRoll();
    // realAccel();

    // blink LED to indicate activity
    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState);
  }
}

void realWorldAccel() {
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  Serial << "aworld\t" << aaWorld.x << "\t" << aaWorld.y << "\t" << aaWorld.z << endl;
}

void realAccel() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  Serial << "areal\t" << aaReal.x << "\t" << aaReal.y << "\t" << aaReal.z << endl;
}

void yawPitchRoll() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  ypr[0] = ypr[0] * 180 / M_PI;
  ypr[1] = ypr[1] * 180 / M_PI;
  ypr[3] = ypr[3] * 180 / M_PI;

  Serial << "ypr\t" << ypr[0] << "\t" << ypr[1] << "\t" << ypr[2] << endl;
}

void dmpDataReady() {
  mpuInterrupt = true;
}