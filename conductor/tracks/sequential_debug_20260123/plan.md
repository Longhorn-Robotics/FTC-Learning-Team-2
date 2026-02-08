# Implementation Plan: Sequential Autonomous Debugging

## Phase 1: Sequential Movement Core (IMU-Based)
- [x] Task: Implement `turnTo(double targetHeading)` using IMU for absolute orientation with real-time telemetry of heading error and motor power. [71f7709]
- [ ] Task: Implement `driveStraight(double inches, double targetHeading)` using IMU for active steering correction and encoders for distance, with live telemetry of drift correction.
- [ ] Task: Conductor - User Manual Verification 'Phase 1: Sequential Movement Core' (Protocol in workflow.md)

## Phase 2: Vision & Hybrid Refinement
- [ ] Task: Filter AprilTag detections by ID (Target Tag: 24 at 45°) and integrate visual distance into `driveStraight` as the primary range source.
- [ ] Task: Add "Visual Re-Lock" logic to update the robot's perceived position whenever our specific tag is seen, with telemetry confirming visual vs. sensor modes.
- [ ] Task: Conductor - User Manual Verification 'Phase 2: Vision & Hybrid Refinement' (Protocol in workflow.md)

## Phase 3: Sequential State Machine
- [ ] Task: Refactor the main loop into a sequential sequence (Turn -> Drive -> Turn), replacing the simultaneous hybrid step.
- [ ] Task: Calibrate specific field angles (0° Lane, 90° Rows, 45° Tag) and validate transitions with detailed state telemetry.
- [ ] Task: Conductor - User Manual Verification 'Phase 3: Sequential State Machine' (Protocol in workflow.md)

## Phase 4: End-Game & Safety
- [ ] Task: Integrate 30s timer checks into the sequential states to ensure the "Safe Zone" exit is never blocked by a long drive task.
- [ ] Task: Final pass on telemetry formatting to ensure the "Robot's understanding of its position" is clearly visible on the Driver Station.
- [ ] Task: Conductor - User Manual Verification 'Phase 4: End-Game & Safety' (Protocol in workflow.md)
