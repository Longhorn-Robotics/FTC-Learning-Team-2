# Specification: Field-Centric Localization System

## Overview
Implement a robust, field-centric localization system for the FTC DECODE season. The system will track the robot's absolute position on the field using (X, Y) coordinates and an absolute heading (Degrees). This replaces the "relative to tag" distance/bearing model with a global coordinate framework.

## Functional Requirements
1.  **Field-Centric Pose Tracking:**
    -   Maintain a global state for the robot's pose: `(fieldX, fieldY, fieldHeading)`.
    -   **fieldHeading:** Absolute angle relative to the goal wall (0 degrees).
    -   **fieldX / fieldY:** Absolute coordinates in inches relative to a fixed field origin (e.g., center of the field or a specific corner).

2.  **Hybrid Localization Strategy (Priority Override):**
    -   **Visual Initialization/Correction:** Use AprilTag detections to initialize the robot's position and continuously correct drift when the tag is visible.
    -   **Dead Reckoning (Fallback):** When the AprilTag is lost, use IMU (gyroscope/compass) for heading and encoders/IMU (accelerometer) for tracking displacement in (X, Y).

3.  **Sensor Integration:**
    -   **Primary:** AprilTag visual data for high-confidence absolute positioning.
    -   **Secondary:** IMU Gyroscope for precise heading.
    -   **Tertiary:** Encoders and IMU Accelerometer for tracking relative movement.

4.  **Navigation Integration:**
    -   Update existing autonomous movement methods to target specific field coordinates (e.g., `driveToCoordinate(x, y, heading)`) instead of relative ranges.

## Non-Functional Requirements
- **High Visibility:** Real-time telemetry output of the current (X, Y, Heading) pose and the source of truth (Visual vs. Sensor).
- **Smoothness:** Logic to prevent "jumps" in position when AprilTag data is first re-acquired.

## Acceptance Criteria
- [ ] Robot can initialize its starting position automatically via AprilTag.
- [ ] Robot accurately maintains its (X, Y) position within 2 inches during short periods of sensor fallback.
- [ ] Autonomous routine successfully executes using coordinate-based targets.

## Out of Scope
- Integration of specialized external localization hardware (e.g., separate Dead-Wheel Odometry modules) unless existing hardware is insufficient.
