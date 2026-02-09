# Implementation Plan: Sequential Autonomous Debugging

## Phase 1: Sequential Movement Core (IMU-Based) [checkpoint: f3ad815]
- [x] Task: Implement `turnTo(double targetHeading)` using IMU for absolute orientation with real-time telemetry of heading error and motor power. [08a0aaa]
- [x] Task: Implement `driveStraight(double inches, double targetHeading)` using IMU for active steering correction and encoders for distance, with live telemetry of drift correction. [08a0aaa]
- [x] Task: Conductor - User Manual Verification 'Phase 1: Sequential Movement Core' (Protocol in workflow.md)

## Phase 2: Vision & Hybrid Refinement
- [ ] Task: Filter AprilTag detections by ID (Target Tag: 24 at 45°) and integrate visual distance into `driveStraight`.
- [ ] Task: Implement "Visual Re-Lock" logic to update the robot's perceived position whenever Tag 24 is seen.
- [ ] Task: Conductor - User Manual Verification 'Phase 2: Vision & Hybrid Refinement' (Protocol in workflow.md)

## Phase 3: Sequential State Machine Implementation
- [ ] Task: Refactor the main loop into the 11-step sequence defined in `spec.md`.
- [ ] Task: Calibrate specific field angles (Tag=45°, Up=0°, Right=90°) and validate transitions with detailed telemetry.
- [ ] Task: Conductor - User Manual Verification 'Phase 3: Sequential State Machine Implementation' (Protocol in workflow.md)

## Phase 4: End-Game & Safety
- [ ] Task: Integrate 25s timer check to trigger launch zone exit.
- [ ] Task: Final pass on telemetry formatting to ensure "Robot's understanding of its position" is visible.
- [ ] Task: Conductor - User Manual Verification 'Phase 4: End-Game & Safety' (Protocol in workflow.md)
