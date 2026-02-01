# Implementation Plan: Field-Centric Localization

## Phase 1: Coordinate System & Math
- [x] Task: Implement `Pose` class (or structure) to hold `(x, y, heading)`. [c8a2b0d]
- [x] Task: Implement `getGroundDistance(tagRange)` logic to account for the 25" tag height. [91f12bc]
- [ ] Task: Implement coordinate transformation logic to convert "Relative Tag 24" and "Relative Tag 20" observations into absolute Field Coordinates.
- [ ] Task: Conductor - User Manual Verification 'Phase 1: Coordinate System & Math' (Protocol in workflow.md)

## Phase 2: Sensor Integration
- [ ] Task: Update `RobotAutonomousFinal` to initialize pose using the first visual detection.
- [ ] Task: Implement `updatePose()` method that merges Encoder/IMU deltas into the global pose.
- [ ] Task: Implement visual correction logic: When a tag is seen, override the "Dead Reckoning" pose with the "Visual" pose.
- [ ] Task: Conductor - User Manual Verification 'Phase 2: Sensor Integration' (Protocol in workflow.md)

## Phase 3: Navigation Update
- [ ] Task: Refactor `driveToTagHybrid` to `driveToCoordinate(targetX, targetY, targetHeading)`.
- [ ] Task: Update the state machine to use coordinate-based targets (e.g., Row 1 is at `(36, 12)`).
- [ ] Task: Conductor - User Manual Verification 'Phase 3: Navigation Update' (Protocol in workflow.md)
