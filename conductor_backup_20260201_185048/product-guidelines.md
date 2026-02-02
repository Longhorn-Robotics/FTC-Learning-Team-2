# Product Guidelines

## Development Priorities
1.  **Safety & Stability:**
    -   Robot movement must be predictable and controlled.
    -   Include "abort" conditions for critical failures (e.g., total sensor loss).
    -   Ensure all motors/servos default to a safe state (zero power/neutral position) upon initialization and termination.

2.  **High Visibility (Telemetry):**
    -   **Real-Time Logging:** The code must generate comprehensive telemetry logs at every loop iteration.
    -   **Required Data Points:**
        -   Current Autonomous State (e.g., `DRIVE_TO_ROW`, `ALIGN_TO_TAG`).
        -   Live Sensor Readings: IMU heading, Encoder positions, AprilTag range/bearing/ID.
        -   Perceived Robot Pose: The robot's calculated "best guess" position (Hybrid x/y coordinates).
        -   Cycle Status: Current row index, ball count (0-2).
    -   **Purpose:** Enable immediate debugging during testing and detailed post-match analysis.

## Operational Logic
1.  **Capacity Management:**
    -   **Strict Limit:** The intake logic must strictly enforce a 2-ball maximum capacity.
    -   **Trigger:** Upon detecting/collecting the 2nd ball, immediately cease collection and transition to the "Return & Score" phase, regardless of the remaining distance in the row.

2.  **Time Management (End-Game):**
    -   **Conservative Exit:** Continuously monitor the 30-second autonomous timer.
    -   **Threshold:** If the remaining time drops below a safety threshold (e.g., 5 seconds), **ABORT** the current collection/scoring cycle immediately.
    -   **Action:** Prioritize moving the robot to a designated "Safe Zone" outside the launch area to avoid penalties.

## Code Structure
-   **State Machine:** Use a clear, enum-based state machine to manage the robot's logic flow.
-   **Helper Methods:** Encapsulate complex logic (e.g., `driveToTagHybrid`, `driveStraight`) into distinct, reusable methods to maintain readability and modularity.
