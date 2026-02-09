# Specification: Sequential Autonomous Debugging

## Goal
Refactor and debug `RobotAutonomousFinal.java` into a robust sequential routine. The system will use discrete "Turn" and "Drive" actions, with AprilTag ID 24 as the primary global anchor.

## Coordinate System (Field-Centric)
Based on `instructions.md`:
-   **Reference:** Top-down view of the court.
-   **0° (Top):** Facing the Top Wall (where goals are located).
-   **45° (Top-Right):** Facing our Target AprilTag (ID 24).
-   **90° (Right):** Facing the Right Wall (perpendicular to ball rows).
-   **Rotation:** Clockwise (CW) Positive. (Note: IMU code must invert raw readings to match).

## Robot Action Sequence
1.  **Initial Align:** Face Tag (45°) and calibrate IMU offset. The robot will already be facing this rough general direction.
2.  **Drive to Lane Entry:** Move along the 45° diagonal to reach the lane column.
3.  **Turn Up:** Rotate to face 0°.
4.  **Reverse to Row:** Drive backward (while facing 0°) down the lane to the target row depth.
5.  **Turn to Balls:** Rotate CW to 90° to face the ball row.
6.  **Intake Run:** Drive forward into the row to collect balls (Max 2).
7.  **Reverse out:** Back up to the lane column (90° heading).
8.  **Turn Up Return:** Rotate CCW back to 0°.
9.  **Return to Lane Entry:** Drive forward to the lane start point.
10. **Final Align:** Rotate CW to face Tag (45°) for scoring.
11. **Repeat/Exit:** Execute cycles until 25s mark, then exit the launch zone.

## Core Requirements
1.  **Sequential Helpers:** Implement distinct `turnTo(heading)` and `driveStraight(distance, heading)` methods.
2.  **Hybrid Logic:** Use Tag 24 range for absolute error when visible; fallback to Encoders/IMU when lost.
3.  **Configurable Routing:** Store `TAG_HEADING`, `UP_HEADING`, `ROW_HEADING`, and distances as top-level variables.

## Non-Functional Requirements
-   **Continuous Telemetry:** Log current State, Heading, Error, Tag ID, and Fallback Mode at every iteration.
-   **Safety:** Proportional slowing to prevent overshoot.