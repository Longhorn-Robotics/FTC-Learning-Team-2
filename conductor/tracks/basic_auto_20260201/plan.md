# Implementation Plan: Basic Autonomous Routine

## Phase 1: Foundation & Tuning
- [ ] Task: Create PedroPathing configuration class
    - [ ] Define `@Config` parameters for tank drive PID, speed, and acceleration.
    - [ ] Initialize PedroPathing with `RobotHardware` constants.
- [ ] Task: Create `BasicAutoOpMode` skeleton
    - [ ] Initialize `RobotHardware` and PedroPathing.
    - [ ] Set starting pose.
- [ ] Task: Conductor - User Manual Verification 'Phase 1: Foundation & Tuning' (Protocol in workflow.md)

## Phase 2: Trajectory Implementation
- [ ] Task: Define scoring trajectories
    - [ ] Path from start to scoring zone.
    - [ ] Path from scoring zone to second ball.
    - [ ] Path back to scoring zone.
- [ ] Task: Implement intake/outtake logic in Auto
    - [ ] Sequential control for `in` and `ou` motors during paths.
- [ ] Task: Conductor - User Manual Verification 'Phase 2: Trajectory Implementation' (Protocol in workflow.md)

## Phase 3: Final Integration & Optimization
- [ ] Task: Combine paths into a state machine routine
    - [ ] Implement transitions between scoring and intaking states.
- [ ] Task: Final autonomous testing and fine-tuning
    - [ ] Adjust pathing constants for maximum reliability.
- [ ] Task: Conductor - User Manual Verification 'Phase 3: Final Integration & Optimization' (Protocol in workflow.md)
