# Specification: Debug Initial Tag Assumption

## Goal
Modify the autonomous routine to assume the AprilTag is visible at the start of the match, allowing the robot to immediately lock on and begin its alignment sequence.

## Requirements
- **Startup Logic:** The robot should not wait or scan for the tag initially; it should expect valid data immediately.
- **Robustness:** Maintain the hybrid navigation fallback if the tag is lost *after* the initial lock.
