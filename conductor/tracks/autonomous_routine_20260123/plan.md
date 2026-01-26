# Implementation Plan: Core Autonomous Routine

## Phase 1: Scaffolding and Setup
- [ ] Task: Create `RobotAutonomousFinal.java` and initialize hardware/vision.
- [ ] Task: Implement comprehensive telemetry helper for real-time sensor/state monitoring.
- [ ] Task: Conductor - User Manual Verification 'Phase 1: Scaffolding and Setup' (Protocol in workflow.md)

## Phase 2: Hybrid Navigation Foundation
- [ ] Task: Implement `driveToTagHybrid` logic for visual tracking with encoder/IMU fallback.
- [ ] Task: Implement `turnToHeading` with IMU normalization (-180 to 180).
- [ ] Task: Conductor - User Manual Verification 'Phase 2: Hybrid Navigation Foundation' (Protocol in workflow.md)

## Phase 3: Collection Logic
- [ ] Task: Implement the "Lane Alignment" sequence to position the robot for row runs.
- [ ] Task: Implement row-specific reversing and 2-ball limit intake logic.
- [ ] Task: Implement return-to-lane-start logic using blind reckoning.
- [ ] Task: Conductor - User Manual Verification 'Phase 3: Collection Logic' (Protocol in workflow.md)

## Phase 4: Scoring and End-Game
- [ ] Task: Implement AprilTag alignment and launch sequence (placeholders for manual code).
- [ ] Task: Implement timer monitoring and the "Safe Zone" exit strategy.
- [ ] Task: Conductor - User Manual Verification 'Phase 4: Scoring and End-Game' (Protocol in workflow.md)
