# BalanceBotUltra Implementation Plan

## Scope
- Hardware target: same Keyestudio REV4 or Arduino Uno plus balance shield used by the reference project.
- v1 behavior: self-balance, discover a usable setpoint from calibration plus trim, and hold near the current position after disturbance instead of returning to startup.
- Relay link: NeoSWSerial on A3/A2 at 9600 baud with structured telemetry and bounded tuning commands.
- Explicitly out of v1: remote drive and steering commands.

## Milestones
1. Project scaffold
   - Create a new PlatformIO project in BalanceBotUltra.
   - Separate hardware, encoder, IMU, control, telemetry, and persistence code.
   - Verify the project compiles before tuning.
2. Hardware bring-up
   - Test serial relay with `C:MODE,TEST_SERIAL` and verify state plus ack frames.
   - Test IMU with `C:MODE,TEST_IMU` and verify angle and gyro signs when leaning the robot.
   - Test motors with `C:MODE,TEST_MOTORS` and confirm actual wheel directions.
   - Test encoders with `C:MODE,TEST_ENCODERS` and manually spin wheels to verify counts and signs.
3. Balance loop
   - Use `C:MODE,BALANCE` or the button to arm the controller.
   - Tune `KP`, `KD`, then `KI` only if needed.
   - Confirm motors cut off when tilt exceeds the configured fault threshold.
4. Hold-position loop
   - Tune `HKP`, `HKI`, and `VKP` after the upright loop is stable.
   - Push the robot and verify it settles near the new location.
5. Persistence
   - Save tuned values with `C:SAVE`.
   - Reboot and verify settings reload.

## Telemetry Frames
- `V:` state frame: mode, armed flag, fault.
- `S:` settings frame: trim, balance gains, hold gains, filter settings, deadband, calibrated zero.
- `T:` runtime frame: angle, angular rates, target angle, hold bias, position error, velocity, pwm, and loop timing.
- `E:` encoder frame: left count, right count, left delta, right delta.
- `F:` event or fault frame.
- `A:` command acknowledgement frame.

## Command Set
- `C:STATUS`
- `C:ARM,1` or `C:ARM,0`
- `C:MODE,IDLE|BALANCE|TEST_IMU|TEST_MOTORS|TEST_ENCODERS|TEST_SERIAL`
- `C:TRIM,<deg>`
- `C:KP,<value>` `C:KI,<value>` `C:KD,<value>`
- `C:HKP,<value>` `C:HKI,<value>` `C:VKP,<value>`
- `C:QA,<value>` `C:QG,<value>` `C:RA,<value>` `C:K1,<value>`
- `C:MINPWM,<0-255>`
- `C:SET,<trim>,<kp>,<ki>,<kd>,<holdKp>,<holdKi>,<velocityKp>,<qAngle>,<qGyro>,<rAngle>,<k1>,<minPwm>`
- `C:RESET`
- `C:KFRESET`
- `C:CLEARFAULT`
- `C:RAWPWM,<left>,<right>` for bench-only direct motor drive while disarmed
- `C:STOP` to stop raw motor drive and return to idle
- `C:ENCRESET` to zero encoder counts before sign testing
- `C:SAVE`

## Test Order
1. Compile and flash with the Bluetooth or relay slide switch in the upload-safe position.
2. Start in `TEST_SERIAL` or `TEST_IMU` before powering motors.
3. Verify wheel direction and encoder sign before trying `BALANCE`.
4. Start balance testing with wheels lifted, then move to supported floor tests.
5. Tune hold-position only after upright correction is stable.
