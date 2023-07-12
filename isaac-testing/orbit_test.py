import torch

from omni.isaac.kit import SimulationApp

config = {"headless": False}
simulation_app = SimulationApp(config)

from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.viewports import set_camera_view

import omni.isaac.orbit.utils.kit as kit_utils
from omni.isaac.orbit.robots.config.anymal import ANYMAL_B_CFG, ANYMAL_C_CFG
from omni.isaac.orbit.robots.config.unitree import UNITREE_A1_CFG
from omni.isaac.orbit.robots.legged_robot import LeggedRobot



# load kit helper
sim = SimulationContext(physics_dt=0.005, rendering_dt=0.005, backend="torch")
set_camera_view(eye=[2.5, 2.5, 2.5], target=[0.0, 0.0, 0.0])

# create robot instance
robot_a = LeggedRobot(cfg=UNITREE_A1_CFG)
robot_b = LeggedRobot(cfg=ANYMAL_B_CFG)
robot_c = LeggedRobot(cfg=ANYMAL_C_CFG)
# spawn the robot
robot_a.spawn("/World/A1", translation=(1.0, 0, 0.42))
robot_b.spawn("/World/AnymalB", translation=(0.0, -0.5, 0.65))
robot_c.spawn("/World/AnymalC", translation=(0.0, 0.5, 0.65))
# spawn ground
# Ground-plane
kit_utils.create_ground_plane(
    "/World/defaultGroundPlane",
    static_friction=0.5,
    dynamic_friction=0.5,
    restitution=0.8,
    improve_patch_friction=True,
)

# play the simulation
sim.reset()
# configure physics handles
for robot in [robot_a, robot_b, robot_c]:
    # initialize the robot
    robot.initialize()
    # reset buffers
    robot.reset_buffers()

# create zero actions
actions = torch.zeros(robot_a.count, robot_a.num_actions, device=robot_a.device)

# run the simulation
for _ in range(1000):
    # step the robot
    for robot in [robot_a, robot_b, robot_c]:
        robot.apply_action(actions)
    # step the simulation
    sim.step()
    # update buffers
    for robot in [robot_a, robot_b, robot_c]:
        robot.update_buffers(dt=sim.get_physics_dt())