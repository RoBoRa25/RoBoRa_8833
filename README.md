# RoBoRa_8833

Arduino library for the ESP32-C3 to drive two DC motors using the **DRV8833** H-bridge.
It supports direct control (`setSpeedA/B`), braking (`brake*`), coasting (`coast*`), and
differential mixing (**tank** / **arcade**) with deadzone, expo and configurable parameters.

## Installation
1. Download the library ZIP.
2. In Arduino IDE: **Sketch > Include Library > Add .ZIP Library** and select the ZIP.

## Quick example
```cpp
#include <RoBoRa_8833.h>

// Motor A and B config (pins + PWM channels)
RoBoRa_8833::MotorCfg motorA = { 5, 6, 0, 1, false };
RoBoRa_8833::MotorCfg motorB = { 7, 8, 2, 3, false };

RoBoRa_8833 driver(motorA, motorB, ROBORA_CTRL_FAST);

void setup() {
  Serial.begin(115200);
  if (driver.begin()) {
    Serial.println("Driver initialized");
  }
  driver.setSpeedA(150);
  driver.setSpeedB(-150);
}

void loop() {
  driver.driveTank(100, 20); // differential mix
  delay(1000);
  driver.brakeAll();
  delay(1000);
}
```

## Layout
```
RoBoRa_8833/
├─ examples/
│  └─ BasicUsage/BasicUsage.ino
├─ src/
│  ├─ RoBoRa_8833.h
│  └─ RoBoRa_8833.cpp
└─ library.properties
```
