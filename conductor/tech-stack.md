# Tech Stack

- **Programming Language:** Java (JDK 21 compatible)
- **Framework:** FIRST Tech Challenge (FTC) SDK for the DECODE (2025-2026) competition season.
- **Platform:** Android-based Robot Controller (Control Hub).
- **Build System:** Gradle (as configured in `build.gradle` and `settings.gradle`).
- **Computer Vision:**
    - **VisionPortal:** Core API for camera management and processor orchestration.
    - **AprilTag Library:** For precise absolute localization and alignment.
    - **OpenCV / ColorBlobLocator:** For detecting and tracking field artifacts (balls) using color thresholds.
- **Hardware Integration:**
    - **IMU:** Integrated 9-axis sensor for absolute heading and orientation fallback.
    - **Encoders:** Motor-integrated encoders for relative distance tracking and dead-reckoning.
    - **RobotHardware Class:** Pre-defined abstraction for motor and servo management.
