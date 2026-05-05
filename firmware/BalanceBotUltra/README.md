# BalanceBotUltra

BalanceBotUltra is a new PlatformIO firmware for the Keyestudio self-balancing car hardware used by the existing balance project in this workspace. It does not copy the reference files, but it reuses the same validated pin mapping and the same core architectural constraints: Uno timing limits, MPU6050 posture sensing, hall encoder polling compatible with NeoSWSerial, and structured telemetry over the relay link.

The initial implementation includes staged bring-up modes, a micros-scheduled balance loop, a position-hold outer loop that can settle at the robot's new location after a push, and EEPROM-backed tuning persistence.

See [docs/implementation_plan.md](docs/implementation_plan.md) for the staged bring-up workflow and command set.