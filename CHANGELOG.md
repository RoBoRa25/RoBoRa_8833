# Changelog

All notable changes to this project will be documented in this file.  
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)  
and this project adheres to [Semantic Versioning](https://semver.org/).

---

## [1.0.0] - 2025-09-25
### Added
- Initial release of **RoBoRa_8833**
- Class `RoBoRa_8833` to drive two DC motors via DRV8833 on ESP32-C3
- Core methods:
  - `setSpeedA/B()` – direct speed control
  - `brake*()` – active braking
  - `coast*()` – coasting
  - `driveTank()` – differential (tank/arcade) mixing
- Configurable parameters: deadzone, expo, arcade, inversions, gain
- Example `BasicUsage.ino`
- `library.properties`, `README.md`, and `LICENSE`
