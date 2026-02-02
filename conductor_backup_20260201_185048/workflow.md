# Project Workflow

## Guiding Principles

1. **The Plan is the Source of Truth:** All work must be tracked in `plan.md`.
2. **The Tech Stack is Deliberate:** Changes to the tech stack must be documented in `tech-stack.md` *before* implementation.
3. **Manual Hardware Verification:** Code is verified through deployment and testing on the physical robot controller.
4. **User Experience First:** Every decision should prioritize autonomous routine precision and reliability.
5. **Non-Interactive & CI-Aware:** Prefer non-interactive commands.

## Task Workflow

All tasks follow a strict lifecycle:

### Standard Task Workflow

1. **Select Task:** Choose the next available task from `plan.md` in sequential order.

2. **Mark In Progress:** Before beginning work, edit `plan.md` and change the task from `[ ]` to `[~]`.

3. **Implement Functionality:**
   - Write the code to fulfill the task requirements.
   - Adhere to the code style guides in `conductor/code_styleguides/`.

4. **Document Deviations:** If implementation differs from the tech stack:
   - **STOP** implementation.
   - Update `tech-stack.md` with the new design.
   - Add a dated note explaining the change.
   - Resume implementation.

5. **Commit Code Changes:**
   - Stage all code changes related to the task.
   - Propose a clear, concise commit message (e.g., `feat(auto): Implement AprilTag hybrid tracking`).
   - Perform the commit.

6. **Attach Task Summary with Git Notes:**
   - **Step 6.1: Get Commit Hash:** Obtain the hash of the *just-completed commit* (`git log -1 --format="%H"`).
   - **Step 6.2: Draft Note Content:** Create a detailed summary including the task name, changes made, and the "why".
   - **Step 6.3: Attach Note:** Use `git notes add -m "<note content>" <commit_hash>`.

7. **Record Task Commit SHA:**
   - **Step 7.1: Update Plan:** Update the task status in `plan.md` from `[~]` to `[x]` and append the first 7 characters of the commit hash.
   - **Step 7.2: Write Plan:** Write the updated content back to `plan.md`.

8. **Commit Plan Update:**
   - Stage and commit the modified `plan.md` (e.g., `conductor(plan): Mark task 'X' as complete`).

### Phase Completion Verification and Checkpointing Protocol

**Trigger:** Executed after completing a task that concludes a phase in `plan.md`.

1.  **Announce Protocol Start:** Inform the user the phase is complete and manual verification is required.

2.  **Propose Manual Verification Plan:**
    - Analyze `product.md` and `plan.md` to determine the goals.
    - Generate a step-by-step verification plan for deployment to the robot.
    - **Format:**
        ```
        The phase implementation is complete. Please follow these steps for manual verification:
        1. **Deploy the code to the robot controller.**
        2. **Run the autonomous routine: [Routine Name]**.
        3. **Confirm that [Expected Behavior] occurs.**
        ```

3.  **Await Explicit User Feedback:**
    - Ask: "**Does this meet your expectations? Please confirm with yes or provide feedback.**"
    - **PAUSE** and await response.

4.  **Create Checkpoint Commit:**
    - Stage all changes and commit (e.g., `conductor(checkpoint): Checkpoint end of Phase X`).

5.  **Attach Verification Report using Git Notes:**
    - Draft a report including the manual steps and the user's confirmation, then attach via `git notes`.

6.  **Record Phase Checkpoint SHA:**
    - Update the phase heading in `plan.md` with `[checkpoint: <sha>]`.

7.  **Commit Plan Update:**
    - Stage and commit `plan.md`.

### Quality Gates

Before marking any task complete, verify:
- [ ] Code follows project's style guidelines.
- [ ] All public methods are documented.
- [ ] Logic respects the 2-ball capacity limit.
- [ ] Telemetry is comprehensive for debugging.

## Commit Guidelines

### Message Format
```
<type>(<scope>): <description>
```

### Types
- `feat`: New autonomous feature.
- `fix`: Bug fix.
- `docs`: Documentation only.
- `style`: Formatting.
- `refactor`: Code reorganization.
- `chore`: Maintenance.

## Definition of Done

A task is complete when:
1. Code is implemented to specification.
2. Changes are committed with a proper message.
3. Git note with task summary is attached.
4. `plan.md` is updated and committed.