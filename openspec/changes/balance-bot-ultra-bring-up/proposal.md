## Why

The BalanceBotUltra firmware has a strong module structure and implementation plan, but key milestones are still scaffolded rather than fully integrated and verified. Completing bring-up now will convert architecture into a testable balancing system and reduce integration risk across firmware, telemetry relay, and dashboard tooling.

## What Changes

- Implement staged firmware bring-up flow for BalanceBotUltra from bench test modes to closed-loop balancing.
- Define and stabilize runtime modes for bench testing, telemetry validation, and balancing operation.
- Complete end-to-end telemetry and command behavior needed for tuning and diagnostics.
- Add guardrails for motor safety, sensor validity, and fail-safe transitions between modes.
- Provide operator-facing tuning workflow compatible with existing serial/WebSocket relay and dashboard consumers.

## Capabilities

### New Capabilities
- `bring-up-runtime-modes`: Deterministic startup and mode transitions covering serial test, motor test, IMU test, and balance mode.
- `balance-control-loop`: Closed-loop balancing using filtered IMU state, encoder feedback, and configurable PID terms.
- `telemetry-command-contract`: Structured telemetry frames and command handling for gains, trims, state visibility, and control actions.
- `safety-and-failsafe`: Motor output limits, invalid sensor handling, emergency stop behavior, and safe mode fallback.
- `field-tuning-workflow`: Runtime parameter tuning and persistence flow that supports iterative tuning from desktop tooling.

### Modified Capabilities
- None.

## Impact

- Affected firmware: `firmware/BalanceBotUltra/src` modules and integration in main runtime loop.
- Affected communication path: serial relay and WebSocket consumers that parse telemetry and issue tuning commands.
- Affected control app behavior: dashboard views/services that display state and send control updates.
- Technical risk areas: timing stability, sensor calibration quality, loop saturation behavior, and command/telemetry compatibility during iteration.