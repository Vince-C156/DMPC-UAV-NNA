#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omniisaacgymenvs.tasks.crazyflie import CrazyflieTask
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

world = World()
world.scene.add_default_ground_plane()
# Add a robot to the scene

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    # Use carb to log warnings, errors and infos in your application (shown on terminal)
    carb.log_error("Could not find nucleus server with /Isaac folder")
asset_path = assets_root_path + "/Isaac/Robots/Crazyflie/cf2x.usd"
# This will create a new XFormPrim and point it to the usd file as a reference
# Similar to how pointers work in memory
#add_reference_to_stage(usd_path=asset_path, prim_path="/World/crazyflie")

#usd_path = 'dmpc-cfly.project.usd'
#add_reference_to_stage(usd_path=usd_path, prim_path="/World/cf2x")
#uav = world.scene.add(Robot(prim_path="/World/crazyflie", name="crazyflie"))
world.add_task(task=CrazyflieTask(name='uav'))  # omni.isaac.core.tasks.base_task.BaseTask
task_params = world.get_task("uav").get_params()
print(task_params)
uav = world.scene.get_object(task_params["Crazyflie"]["value"])
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly


