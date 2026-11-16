https://docs.robotis.com/docs/systems/openmanipulator_x/overview/

Task 3: The Wire‑Trace Trial
October 17, 2025 1:11 PM
November 16, 2025
Debojit Das
Computer Vision
Path Planning
Trajectory Planning
Inverse Kinematics
Forward Kinematics
Worksapce Analysis
Trial Three of the Tri‑Wizard Robotics Endgame stands on its own. This challenge is independent of Task 1 and Task 2, but follows the same spirit. Your quest: trace a vertical wire path but with a robot.
The tale so far
A wire loop stands vertically in the workspace, twisted into a challenging 3D path. Your mission: guide the robot's end‑effector (or a probe attached to it) along the wire without making contact. Think of it as the classic "buzz wire" game, but with a robot arm and precise trajectory control.
Robot: Addverb Heal
Scene: Wire loop mounted vertically on a stand, with twists and turns.
Challenge: Trace the wire path as quickly and accurately as possible without contact.
Quest objectives
Record the wire path: manually guide the robot along the wire using gravity compensation
Process the recorded trajectory: smooth, interpolate, and optimize the waypoints
Execute the trajectory autonomously using DLS-IK and velocity control
Optimize for speed while maintaining zero contact
Share a fully reproducible repository and perform a public demo


Iss bar no complex vision, navigation problem, bas simple Mr. Bean seh better your robot should be able to trace the wire.
Trial map (modify as needed)
Stage 1

Stage 2

Enable Gravity Compensation

Manual Teaching
Guide end-effector along wire

Record Waypoints
Position + orientation

Stage 1 or 2?

Trajectory Processing
Interpolation + smoothing

Learning-Based Approach Both Classical or Non Classical

DLS-IK Solver
Convert to joint commands

Execution
Send velocity commands to robot

Monitor & Verify
Contact detection + timing

​
Stage 1: Gravity compensation and trajectory replay (100 Points)
In this stage, you will use gravity compensation mode to manually guide the robot along the wire, record the trajectory, and then replay it autonomously.
Workflow
Teach phase:
Enable gravity compensation on the robot (this allows you to move the arm freely by hand while the controller compensates for gravity).
Manually guide the end‑effector along the wire path, maintaining a small clearance (2–5 mm).
Record waypoints (positions and orientations) at a suitable sampling rate.
Replay phase:
Feed the recorded waypoints to your DLS‑IK solver.
Generate intermediate trajectory points
Execute the trajectory by sending joint velocity commands to the robot.
Monitor for contact (optional: use a force/torque sensor or audio cues if available).
Sample code:
We will provide sample code to:
Enable gravity compensation mode
Send velocity commands to the robot during replay
Evaluation criteria
Success: Complete the wire trace without contact
Speed: Fastest completion time among all groups wins 10 bonus points
Smoothness: Trajectory should be smooth with no jerky motions or oscillations
Repeatability: Should be able to replay the trajectory multiple times with consistent results
Stage 2: Learning-based trajectory adaptation (Both Classical and Non-Classical Methods) / Any alternative based method that doesn’t involve hardcoding the trajecotry or the robot learns the trajectory over trials and can then do the task in one go. (Bonus - up to 50 points)
Safety protocols
Read and follow these rules strictly. We have only one wire setup—please handle it with care.
Buddy system: Always work in pairs. One team member operates the robot; the other keeps a hand on the emergency stop switch at all times.
Emergency stop: If anything goes wrong (unexpected motion, contact with the wire, etc.), press the emergency stop immediately.
Slow motion first: Test all trajectories at reduced speed (10–20%) before running at full speed.
Workspace limits: Define and enforce software joint limits and Cartesian boundaries. The wire stand is fragile—avoid collisions.
Visual monitoring: Always keep the robot in sight during execution. Do not rely solely on code.
Damage policy: If the wire or stand is damaged due to negligence, your team will be responsible for repair or replacement costs. Follow the safety rules!
Frames and kinematics
Frames: world, base_link, ee. Maintain a TF tree.
Wire frame: Optionally define a wire-local frame for easier path representation.
IK solver: Use damped least‑squares IK (DLS‑IK) with appropriate damping factor to avoid singularities and ensure smooth motion.
Trajectory processing and generation
Interpolation: Use cubic spline, quintic polynomial, or trapezoidal velocity profile to generate smooth trajectories between waypoints.
Clearance: Offset the trajectory radially outward from the wire by a small margin (2–5 mm) to avoid contact.
Velocity tuning: Start with low velocity (5–10 cm/s) and increase gradually as confidence grows.
sequenceDiagram
    participant U as Wizard (You)
    participant P as Trajectory Processor
    participant R as Robot Arm
    U->>R: Enable gravity compensation mode
    U->>R: Manually guide arm along wire (record waypoints)
    R-->>U: Waypoints saved
    U->>P: Process recorded trajectory (smooth + interpolate)
    P-->>U: Processed trajectory ready
    U->>R: Send velocity/position commands
    R-->>U: Execution status (success/contact/time)
Robot Arm
Trajectory Processor
Wizard (You)
Robot Arm
Trajectory Processor
Wizard (You)
Enable gravity compensation mode
Manually guide arm along wire (record waypoints)
Waypoints saved
Process recorded trajectory (smooth + interpolate)
Processed trajectory ready
Send velocity/position commands
Execution status (success/contact/time)
​
Deliverables of power
GitHub repository with full replication steps
Public demonstration of Stage 1 (required) and Stage 2 (bonus)
Video submission (3 mins max) showing teaching, replay, and timing
No report required — the README is your spell scroll.
README checklist

Setup instructions: OS, versions, dependencies, robot drivers

Launch files or scripts for gravity compensation mode and trajectory replay

Instructions for teaching phase: how to enable gravity comp, record waypoints

Instructions for replay phase: how to load waypoints, process trajectory, run DLS‑IK, execute

Sample code for sending velocity commands to the robot

Safety checklist and emergency stop procedure

Evaluation script to log time, contact events, and success rate

(Stage 2 only) Description of learning approach and instructions to run it
Judging rubric (Total: 100 + 30 bonus)
Category
Points
Notes
Stage 1: Successful trace (no contact)
50
Complete the wire path without touching it
Stage 1: Smooth trajectory
20
No jerky motions, well-tuned velocity profile
Stage 1: Repeatability
10
Consistent results over 3 trials
GitHub README and reproducibility
10
Clear setup, launch, and replication steps
Safety compliance
10
Buddy system, E-stop readiness, no damage
Bonus: Fastest execution
10
Shortest time among all groups (Stage 1)
Bonus: Stage 2 — One-shot learning
20
Learning-based approach with minimal hardcoding
Note: Bonus points are cumulative and can push your total above 100.
Victory condition
Stage 1: Trace the entire wire path without contact in the shortest time possible. Complete three successful runs to demonstrate repeatability.
Stage 2 (Bonus): Demonstrate a learning-based approach that requires minimal manual input and generalizes to small variations in the wire configuration.
Announce your triumph with a public demo and a pristine repository so any apprentice can reproduce the magic.
Tips from past wizards
Start slow: Test at 10% speed first, then gradually increase. Speed comes with confidence.
Tune damping: A good damping factor in DLS‑IK prevents oscillations near singularities.
Record densely: More waypoints = smoother replay. Aim for 10–20 Hz sampling during teaching.
Monitor contact: If you have a force/torque sensor, set a low threshold (e.g., 5 N) to detect accidental contact.
Practice teaching: The quality of your manual demonstration directly affects replay performance. Smooth, steady motions work best.
Visualize: Use RViz or similar tools to visualize the planned trajectory before execution.
May your trajectories be smooth, your IK solver stable, and your wire untouched. Good luck, wizard!
Make this page full like an assignment !
References
youtube.com/watch?v=og-xde1ImOk&feature=youtu.be
https://math.libretexts.org/Workbench/Numerical_Methods_with_Applications_(Kaw)/5%3A_Interpolation/5.05%3A_Spline_Method_of_Interpolation
https://en.wikipedia.org/wiki/Quintic_equation
https://en.wikipedia.org/wiki/Spherical_linear_interpolation
