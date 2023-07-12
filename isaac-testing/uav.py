from omni.isaac.examples.base_sample import BaseSample #boiler plate of a robotics extension application
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import carb

class crazy_sample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    # This function is called to setup the assets in the scene for the first time
    # Class variables should not be assigned here, since this function is not called
    # after a hot-reload, its only called to load the world starting from an EMPTY stage
    def setup_scene(self):
        
        # A world is defined in the BaseSample, can be accessed everywhere EXCEPT __init__
        world = self.get_world()
        world.scene.add_default_ground_plane() # adds a default ground plane to the scene

        # Add a robot to the scene
        usd_path = 'dmpc-cfly.project.usd'
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/cf2x")

        uav = world.scene.add(Robot(prim_path="/World/cf2x", name="crazyflie"))
        print("Num of degrees of freedom before first reset: " + str(crazyfliequad.num_dof))
        return
    
    async def setup_post_load(self):
        self._world = self.get_world()
        self._uav = self._world.scene.get_object("crazyflie")
        # Print info about the jetbot after the first reset is called
        print("Num of degrees of freedom after first reset: " + str(self._uav.num_dof)) # prints 2
        print("Joint Positions after first reset: " + str(self._uav.get_joint_positions()))
        return