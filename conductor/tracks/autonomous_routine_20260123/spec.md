# Specification: Core Autonomous Routine

## Goal
Implement a robust autonomous routine for the DECODE season that navigates to ball rows, collects a maximum of 2 balls, scores them in the hoop using AprilTag alignment, and exits the launch zone before time expires.

## Requirements
- **Hardware Integration:** Use `RobotHardware` class for movement and intake/launcher control.
- **Localization:**
    - Absolute: AprilTag detection (45-degree corner tag).
    - Relative: IMU (heading) and Encoders (distance).
- **Phases:**
    1. **Lane Alignment:** Reach the starting point for the collection lane.
    2. **Collection Cycle:** Reverse to row -> Turn to balls -> Intake (Max 2) -> Return to lane start.
    3. **Scoring:** Align to AprilTag -> Launch.
    4. **End-Game:** Move to "Safe Zone" if timer < 5s.

## Constraints
- **Capacity:** Max 2 balls.
- **Penalty:** Must not be in launch zone at T-0.
- **Visibility:** Handle periods where AprilTag is lost using Dead Reckoning.
