# Specification: Sequential Autonomous Debugging

## Goal
Refactor and debug `RobotAutonomousFinal.java` into a robust sequential routine. The system will use discrete "Turn" and "Drive" actions, with AprilTag ID 24 as the primary global anchor and IMU/Encoders as fallbacks.

## Core Requirements
1.  **Sequential Movement:**
    -   Replace the simultaneous hybrid step with distinct method calls: `turnTo(double targetHeading)` and `driveStraight(double inches, double targetHeading)`.
    -   `driveStraight` must include active heading correction (Lock Heading) using IMU/Tag bearing.

2.  **Localization & Anchoring:**
    -   **Target Tag:** Strictly filter for AprilTag ID 24 (45° corner).
    -   **Hybrid Logic:** 
        -   **Visual:** Use Tag 24 range for absolute error.
        -   **Fallback:** If Tag 24 is lost, use relative Encoders + IMU to track remaining distance and heading.
        -   **Re-Lock:** Seamlessly resume visual tracking when Tag 24 reappears.

3.  **Configurable Routing (Variables):**
    -   All field locations must be stored as variables at the top of the file:
        -   `TAG_ID = 24`
        -   `LANE_ENTRY_DISTANCE` (e.g., 36.0")
        -   `ROW_DEPTHS[]` (e.g., {12.0, 36.0, 60.0, 84.0})
        -   `COLLECTION_DEPTH` (e.g., 24.0")
        -   `SCORE_DISTANCE` (e.g., 12.0")

4.  **Operational Logic:**
    -   **2-Ball Limit:** Implement a distance-based collection cycle (State transitions after a fixed drive distance).
    -   **End-Game:** Continuous timer monitoring to ensure "Safe Zone" exit if T < 5s.

## Non-Functional Requirements
-   **Continuous Telemetry:** Log State, Heading, Error, Tag ID, and Fallback Mode at every iteration.
-   **Safety:** Proportional slowing (`Error * Gain`) for all movements to prevent overshoot.