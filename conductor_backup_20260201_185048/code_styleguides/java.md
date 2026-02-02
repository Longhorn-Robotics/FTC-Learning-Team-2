# Java Code Style Guide (FTC/Android Studio)

## Naming Conventions
- **Classes:** PascalCase (e.g., `RobotAutonomous`, `HardwareMap`).
- **Methods:** camelCase (e.g., `runOpMode`, `driveToTag`).
- **Variables:** camelCase (e.g., `leftMotor`, `targetDistance`).
- **Constants:** UPPER_SNAKE_CASE (e.g., `MAX_SPEED`, `COUNTS_PER_INCH`).

## Formatting
- **Indentation:** 4 spaces (standard Android Studio default).
- **Braces:** K&R style (opening brace on the same line).
- **Imports:** Explicit imports preferred over wildcards (`*`).

## Best Practices
- **Hardware Access:** Use the `hardwareMap` to retrieve devices in the `init` phase, never in the loop.
- **Safety:** Always verify objects (like `AprilTagDetection.metadata`) are not null before accessing their properties.
- **Telemetry:** Use `telemetry.addData()` followed by `telemetry.update()` to persist data to the Driver Station.
- **OpMode Structure:**
    - Use `@Autonomous` or `@TeleOp` annotations to register OpModes.
    - Extend `LinearOpMode` for sequential logic (preferred for autonomous).
    - Ensure `waitForStart()` is called before the main loop.
    - Check `opModeIsActive()` in all loops.

## Comments
- Use Javadoc (`/** ... */`) for class and method descriptions.
- Use inline comments (`//`) for explaining complex logic or "why" a decision was made.
