# Product Definition

## Initial Concept
**Goal:** Develop a robust and precise autonomous routine for the DECODE (2025-2026) FTC season using the Android-based Control Hub. The system prioritizes computer vision (AprilTags) for absolute localization and alignment, seamlessly falling back to dead-reckoning (IMU/Encoders) during known periods of visual occlusion. The routine focuses on reliably collecting and scoring balls from field rows while strictly adhering to the 2-ball storage limit and ensuring the robot exits the launch zone before the autonomous period ends.

## Core Features
- **Hybrid Localization System:**
    - **Primary:** AprilTag detection for absolute positioning and heading correction relative to the 45-degree target hoop.
    - **Secondary (Fallback):** IMU and Encoder-based dead-reckoning for navigation during known blind spots (e.g., reversing down the lane, returning to the start).
- **Intelligent Ball Collection:**
    - Navigate to specific rows (12", 36", 60", 84" depths) using precise distance tracking.
    - Utilize an active intake to collect balls while maintaining correct heading.
    - **Constraint:** Strictly manage a 2-ball capacity, necessitating frequent returns to the scoring zone.
- **Precision Scoring:**
    - Auto-align with the target AprilTag from the designated scoring zone.
    - Launch mechanisms (placeholder code) trigger only when alignment is confirmed.
- **Safety & Compliance:**
    - Ensure the robot is outside the launch zone at the end of the 30-second autonomous period to avoid penalties.

## Strategy: Precision-Focused
- Prioritize clearing rows with high accuracy rather than speed.
- Execute a reliable "cycle" pattern: Align -> Navigate to Row -> Collect (Max 2) -> Return -> Re-align -> Score.
- Abort/Exit logic to vacate the launch zone before T-0.
