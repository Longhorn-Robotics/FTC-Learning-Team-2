# Project Workflow

## Guiding Principles

1. **The Plan is the Source of Truth:** All work must be tracked in `plan.md`
2. **The Tech Stack is Deliberate:** Changes to the tech stack must be documented in `tech-stack.md` *before* implementation
3. **Physical Deployment Testing:** Verification requires deploying and testing code on the physical robot. Automated tests are not required for this project.
4. **User Experience First:** Every decision should prioritize user experience and robot performance.
5. **Non-Interactive & CI-Aware:** Prefer non-interactive commands.

## Task Workflow

All tasks follow a strict lifecycle:

### Standard Task Workflow

1. **Select Task:** Choose the next available task from `plan.md` in sequential order

2. **Mark In Progress:** Before beginning work, edit `plan.md` and change the task from `[ ]` to `[~]`

3. **Implement Functionality:**
   - Write the code necessary to implement the feature or fix the bug.
   - Mimic existing styles and patterns in the codebase.

4. **Physical Verification:**
   - Deploy the code to the robot.
   - Verify that the changes work as expected in the physical environment.

5. **Refactor (Optional but Recommended):**
   - Refactor the implementation code to improve clarity, remove duplication, and enhance performance.

6. **Document Deviations:** If implementation differs from tech stack:
   - **STOP** implementation
   - Update `tech-stack.md` with new design
   - Add dated note explaining the change
   - Resume implementation

7. **Commit Changes with Summary:**
   - Stage all changes related to the task.
   - Create a commit message that includes a clear description of the task and a summary of the changes.
   - Example:
     ```
     feat(auto): Implement PedroPathing for initial ball intake

     - Configured PedroPathing constants for tank drive
     - Added trajectory for moving to the first ball location
     - Updated RobotHardware to include intake motor power settings
     ```
   - Perform the commit.

8. **Get and Record Task Commit SHA:**
    - **Step 8.1: Update Plan:** Read `plan.md`, find the line for the completed task, update its status from `[~]` to `[x]`, and append the first 7 characters of the *just-completed commit's* commit hash.
    - **Step 8.2: Write Plan:** Write the updated content back to `plan.md`.

9. **Commit Plan Update:**
    - **Action:** Stage the modified `plan.md` file.
    - **Action:** Commit this change with a descriptive message (e.g., `conductor(plan): Mark task 'Implement pathing' as complete`).

### Phase Completion Verification and Checkpointing Protocol

**Trigger:** This protocol is executed immediately after a task is completed that also concludes a phase in `plan.md`.

1.  **Announce Protocol Start:** Inform the user that the phase is complete and the verification and checkpointing protocol has begun.

2.  **Propose a Detailed, Actionable Manual Verification Plan:**
    -   **CRITICAL:** To generate the plan, first analyze `product.md`, `product-guidelines.md`, and `plan.md` to determine the user-facing goals of the completed phase.
    -   You **must** generate a step-by-step plan that walks the user through the verification process on the physical robot.
    -   The plan you present to the user **must** follow this format:

        **For a Robot Control Change:**
        ```
        The phase is complete. For manual verification on the robot, please follow these steps:

        **Manual Verification Steps:**
        1.  **Deploy the code to the Control Hub.**
        2.  **Select the OpMode:** `MyAutonomousOpMode`
        3.  **Run the OpMode and confirm:** The robot follows the path accurately and intakes the first ball.
        ```

3.  **Await Explicit User Feedback:**
    -   After presenting the detailed plan, ask the user for confirmation: "**Does this meet your expectations? Please confirm with yes or provide feedback on what needs to be changed.**"
    -   **PAUSE** and await the user's response. Do not proceed without an explicit yes or confirmation.

4.  **Create Checkpoint Commit:**
    -   Stage all changes. If no changes occurred in this step, proceed with an empty commit.
    -   Perform the commit with a clear and concise message (e.g., `conductor(checkpoint): Checkpoint end of Phase X`).

5.  **Get and Record Phase Checkpoint SHA:**
    -   **Step 5.1: Get Commit Hash:** Obtain the hash of the *just-created checkpoint commit* (`git log -1 --format="%H"`).
    -   **Step 5.2: Update Plan:** Read `plan.md`, find the heading for the completed phase, and append the first 7 characters of the commit hash in the format `[checkpoint: <sha>]`.
    -   **Step 5.3: Write Plan:** Write the updated content back to `plan.md`.

6. **Commit Plan Update:**
    - **Action:** Stage the modified `plan.md` file.
    - **Action:** Commit this change with a descriptive message following the format `conductor(plan): Mark phase '<PHASE NAME>' as complete`.

7.  **Announce Completion:** Inform the user that the phase is complete and the checkpoint has been created.

### Quality Gates

Before marking any task complete, verify:

- [ ] Code follows project's code style guidelines (as defined in `code_styleguides/`)
- [ ] All public functions/methods are documented (e.g., Javadoc)
- [ ] No linting or static analysis errors
- [ ] Robot hardware map names match `product-guidelines.md`
- [ ] Documentation updated if needed
- [ ] Changes committed with proper message summary

## Commit Guidelines

### Message Format
```
<type>(<scope>): <description>

[optional body - Use this for task summaries]

[optional footer]
```

### Types
- `feat`: New feature (e.g., new path, new motor control)
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Formatting, missing semicolons, etc.
- `refactor`: Code change that neither fixes a bug nor adds a feature
- `chore`: Maintenance tasks (e.g., updating SDK, changing constants)

## Definition of Done

A task is complete when:

1. All code implemented to specification
2. Physical verification on the robot is successful
3. Code passes all configured linting checks
4. Implementation notes added to the commit message
5. Changes committed with proper message
6. Task marked as complete in `plan.md` with commit SHA