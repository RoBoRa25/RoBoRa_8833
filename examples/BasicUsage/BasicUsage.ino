#include <RoBoRa_8833.h>

// Configurazione motore A e B
RoBoRa_8833::MotorCfg motA = { 5, 6, 0, 1, false };
RoBoRa_8833::MotorCfg motB = { 7, 8, 2, 3, false };

RoBoRa_8833 driver(motA, motB, ROBORA_CTRL_FAST);

void setup() {
  Serial.begin(115200);
  if (driver.begin()) {
    Serial.println("Driver inizializzato!");
  }

  // Avvia i motori
  driver.setSpeedA(200);
  driver.setSpeedB(200);
}

void loop() {
  driver.driveTank(100, 30); // mix throttle e steer
  delay(2000);

  driver.coastAll();
  delay(1000);

  driver.brakeAll();
  delay(1000);
}
