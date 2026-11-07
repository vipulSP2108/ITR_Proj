Tier 1: Synthetic Vision Unit Testing (Zero Hardware Needed)

Before touching a physical camera, you need to prove your shape classification, color segmentation, and orientation math work across all rotations and positions.

How to do it:

Write a script scripts/generate_synthetic_scene.py using OpenCV:
Draw a tabletop canvas with a socket board and the 4 shapes (Square, Triangle, Circle, Hexagon) at random 
(
𝑥
,
𝑦
,
𝜃
)
(x,y,θ) poses.
Generate a corresponding synthetic 16-bit depth map:
Table
=
500
 mm
,
Board
=
480
 mm
,
Peg tops
=
380
 mm
Table=500 mm,Board=480 mm,Peg tops=380 mm
Inject artificial noise: Gaussian blur, HSV hue jitter (
±
5
%
±5%), and salt-and-pepper depth noise.
Automated Assertion Tests:
Does shape_detection.py correctly classify all 4 shapes?
Is the estimated centroid 
(
𝑥
,
𝑦
)
(x,y) within 
±
2
±2 pixels of ground truth?
Is the estimated yaw angle 
𝜃
θ correct modulo symmetry (
90
∘
90
∘
 for square, 
60
∘
60
∘
 for hexagon, 
120
∘
120
∘
 for triangle)?
Tier 2: Kinematics & Workspace Feasibility Testing

Manipulator X is a 4–5 DOF arm. If the planner asks it to reach an impossible pose, the physical robot will fault or stall.

How to do it:

Write analytical or numerical Forward Kinematics (FK) and Inverse Kinematics (IK) functions based on the Manipulator X link lengths.
Round-Trip Unit Tests:
𝑃
target
(
𝑥
,
𝑦
,
𝑧
,
yaw
)
→
IK
𝑞
(
𝜃
1
,
𝜃
2
,
𝜃
3
,
𝜃
4
)
→
FK
𝑃
actual
P
target
	​

(x,y,z,yaw)
IK
	​

q(θ
1
	​

,θ
2
	​

,θ
3
	​

,θ
4
	​

)
FK
	​

P
actual
	​

Verify that 
∥
𝑃
target
−
𝑃
actual
∥
<
0.5
 mm
∥P
target
	​

−P
actual
	​

∥<0.5 mm.
Workspace Heatmap / Collision Test:
Sweep the reachable workspace. If any peg or socket pose is generated outside the reachable spherical shell, verify that the planner raises an UnreachablePoseException gracefully rather than sending illegal joint angles.
Tier 3: Hardware Abstraction Layer (HAL)

To make sure you never have to rewrite your planner or state machine when switching to the physical robot, use a Python abstract interface:

python
# src/control/robot_interface.py
from abc import ABC, abstractmethod
class RobotInterface(ABC):
    @abstractmethod
    def move_to_pose(self, x, y, z, yaw, duration_sec): pass
    @abstractmethod
    def set_gripper(self, open_state: bool): pass
    @abstractmethod
    def get_joint_angles(self): pass
    @abstractmethod
    def emergency_stop(self): pass
MockRobot: Simulates joint motions in memory, logs waypoints, prints trajectory steps, and updates a 3D visualization.
RealManipulatorX: Sends Dynamixel protocol 2.0 packets over /dev/ttyUSB0 to the actual DYNAMIXEL servos.

You can switch between them with a single config flag:

bash
python run.py --mode sim    # Runs Mock / MuJoCo
python run.py --mode real   # Runs Physical Hardware
Tier 4: Physics & Rendered Simulation with MuJoCo

Since your environment already has mujoco 3.11.0 installed, we can simulate the entire physical task:

Scene definition (scene.xml):
Table surface and gravity.
OpenManipulator-X model (URDF/MJCF with 4 arm joints and a 2-finger parallel gripper).
4 peg meshes/primitives with exact dimensions (4–6 cm wide, 10 cm tall) and colors.
Socket board with recessed matching hollows.
A virtual overhead RGB-D camera.
Simulation Loop:
MuJoCo renders the virtual camera's RGB frame (renderer.render()) and depth buffer (renderer.read_depth()).
Your perception pipeline receives these synthetic frames directly.
Your planner calculates trajectories and commands joint positions in MuJoCo.
You can visually watch the simulated arm grasp the peg and insert it into the socket, verifying friction, collision, and tolerances.
Recommended Next Step

We can start setting up the simulation and test framework directly inside T1/:

Set up the recommended repository folder structure (src/vision, src/planning, src/control, config, tests).
Create the synthetic vision test suite with an interactive OpenCV preview so you can verify color segmentation, shape classification, and yaw estimation right now on your screen.
Build the RobotInterface and IK solver with automated unit tests.

Would you like to start by generating the synthetic RGB-D vision test script, or would you prefer setting up the MuJoCo simulation environment first?