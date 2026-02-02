# Technology Stack - FTC Robot Controller

## Core Platform
- **Programming Language:** Java (Standard for FTC SDK)
- **Robot Controller SDK:** FTC SDK (DECODE 2025-2026 Season)
- **Hardware Integration:** REV Expansion Hub

## Navigation & Control
- **Autonomous Pathing:** PedroPathing
- **Drive System:** Tank Drive (2 motors: `ld`, `rd`)
- **Localization:** Integrated Encoders + Expansion Hub IMU (BNO055/BHI260AP)

## Tools & Libraries
- **FTC Dashboard:** For real-time telemetry, configuration tuning (`Config` classes), and visual debugging.
- **PedroPathing Library:** Core dependency for spline-based path following.

## Development Environment
- **IDE:** Android Studio (Ladybug 2024.2 or later)
- **Build System:** Gradle
