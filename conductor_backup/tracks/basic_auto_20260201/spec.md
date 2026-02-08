# Specification: Basic Autonomous Routine

## Overview
Implement an autonomous OpMode for the DECODE 2025-2026 season that scores two balls using PedroPathing. The robot uses a tank drive configuration with encoders and an Expansion Hub IMU.

## Requirements
- **Hardware Integration:** Utilize the existing `RobotHardware.java` with identifiers `ld`, `rd`, `in`, and `ou`.
- **Pathing:** Use PedroPathing to define a trajectory from the starting position to the scoring zone.
- **Scoring Logic:**
    1. Start with one ball stored.
    2. Move to score the first ball.
    3. Move to intake a second ball (respecting the 2-ball limit).
    4. Move to score the second ball.
- **Tuning:** Constants for PedroPathing must be adjustable via FTC Dashboard (`@Config`).

## Success Criteria
- The robot accurately follows the path to the scoring zone.
- Two balls are successfully scored during the autonomous period.
- No storage capacity penalties (max 2 balls).
- Localization remains stable throughout the routine.
