# Product Definition - FTC Robot Controller (DECODE 2025-2026)

## Initial Concept
A high scoring good autonomous using PedroPathing. We have tank drive wheels (and sepcifications about them in the code), encoders, and the IMU on our expansion hub. We can only store two balls at a time.

## Target Audience
- **The Team:** Primary developers and operators of the robot, focusing on a balance of technical maintenance and high-performance competitive play.

## Goals
- **High-Scoring Autonomous:** Deliver a competitive edge during the autonomous period using advanced pathing and blind/encoder-based detection.
- **Efficient Scoring:** Optimize the cycle time for scoring the 2-ball maximum storage capacity.
- **Robust Localization:** Maintain accurate positioning using a combination of encoders and the Expansion Hub IMU.

## Core Features
- **PedroPathing Implementation:** Advanced path following specifically tuned for a tank drive chassis.
- **Tank Drive Optimization:** Custom motion profiles and specifications tailored to the team's drive wheel and encoder setup.
- **Expansion Hub IMU Integration:** Real-time heading correction and stabilization.
- **State Machine Control:** Manage the scoring sequences and ball handling logic efficiently.

## Configuration & Maintenance
- **Dashboard Integration:** Use a `Config` class with `public static` fields for real-time tuning via FTC Dashboard.