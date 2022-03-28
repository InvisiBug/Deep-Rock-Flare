void startMPU() {
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(72);
  mpu.setYGyroOffset(-51);
  mpu.setZGyroOffset(28);
  mpu.setXAccelOffset(-4254);
  mpu.setYAccelOffset(-1984);
  mpu.setZAccelOffset(1442);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (mpu.dmpInitialize() == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(devStatus);
    // Serial.println(F(")"));
  }
}

void readAcceleration() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

int readYawPitchRoll() {
  // uint8_t fifoBuffer[64];  // FIFO storage buffer
  // Quaternion q;            // [w, x, y, z]         quaternion container
  // VectorFloat gravity;     // [x, y, z]            gravity vector

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

bool checkShake() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    readAcceleration();

    int threshold = 10000;

    if ((aaReal.x > threshold || aaReal.x < -threshold) ||
        (aaReal.y > threshold || aaReal.y < -threshold) ||
        (aaReal.z > threshold || aaReal.z < -threshold)) {
      return true;
    }
    return false;
  }
}