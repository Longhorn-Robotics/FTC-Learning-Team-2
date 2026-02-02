# Product Guidelines - FTC Robot Controller

## Code Style & Naming
- **Hardware Map Names:** Use the existing short-code identifiers for the hardware configuration:
    - Left Drive: `"ld"`
    - Right Drive: `"rd"`
    - Intake: `"in"`
    - Launch/Output: `"ou"`
- **Variable Naming:** Use **PascalCase** for hardware variables (e.g., `LDriveMotor`, `IntakeMotor`) to match `RobotHardware.java`.
- **Method Naming:** Use **PascalCase** or **camelCase** consistently with the existing `RobotHardware` methods (e.g., `AutoInit`, `moveRobot`).

## Autonomous Standards (PedroPathing)
- **Pathing:** All autonomous movements must be implemented using `PedroPathing` for consistency and performance.
- **Tuning:** Use the `Config` class for pathing constants (PID, speed, acceleration) to allow for rapid on-field adjustments.

## Telemetry & Feedback
- **Operator Info:** Provide essential robot status (battery, motor power, path completion) using standard telemetry.
- **Diagnostics:** Log detailed sensor data (IMU heading, encoder counts) to the FTC Dashboard for remote debugging.

## Mechanical Constraints
- **Capacity Management:** Logic must strictly enforce the 2-ball maximum storage limit to prevent penalties.
- **Tank Drive:** All motion logic assumes a standard tank drive configuration with two main drive motors (`ld`, `rd`).
