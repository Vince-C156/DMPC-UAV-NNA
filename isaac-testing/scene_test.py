from omni.isaac.kit import SimulationApp
import carb
import omni
import sys
import os
print('Any Omniverse level imports must occur after the class is instantiated. APIs are provided by the extension/runtime plugin system, it must be loaded before they will be available to import')

def load_usd(usd_path : str):
    """
    load_usd loads a usd file into the simulation app

    Args:
        usd_path (str): _description_

    Returns: simulator app object
    """

    # This sample loads a usd stage and starts simulation
    CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}



    kit = SimulationApp(launch_config=CONFIG)


    # make sure the file exists before we try to open it
    try:
        result = os.path.exists(usd_path)
    except:
        result = False

    if result:
        omni.usd.get_context().open_stage(usd_path)
    else:
        carb.log_error(
            f"the usd path {usd_path} could not be opened, please make sure that {usd_path} is a valid usd file"
        )
        kit.close()
        sys.exit()
    # Wait two frames so that stage starts loading
    kit.update()
    kit.update()

    print("Loading stage...")
    from omni.isaac.core.utils.stage import is_stage_loading

    while is_stage_loading():
        kit.update()
    print("Loading Complete")
    omni.timeline.get_timeline_interface().play()
    return kit

def crazyflytask():
    from omniisaacgymenvs.tasks.crazyflie import CrazyflieTask
    my_world = World(stage_units_in_meters=1.0)
    my_task = CrazyflieTask(name="crazyflie")
    env.set_task(my_task, backend="torch")
    
    env._world.reset()
    obs = env.reset()
    while env._simulation_app.is_running():
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)

    env.close()

def crazyflyscene():

    sim_app = load_usd('dmpc-cfly.project.usd')
    from pxr import Usd, UsdGeom
    """
    What is USD? (Universal Scene Description)

    open and extensible framework and ecosystem for describing, composing, simulating and collaborating within 3D worlds,
    originally developed by Pixar Animation Studios. USD is more than a file format, it’s an open source 3D scene description used for 3D content creation and interchange among different tools.
    As a result of its power and versatility, it’s being widely adopted, not only in the visual effects community, but also in architecture, design, robotics, manufacturing, and other industries.


    USD, can be seen as a nested file type that can contain other file types, such as OBJ, Alembic, FBX, and USD itself. Generally the outermost scope of the USD file typically defines characteristics of a scene such as the lighting, ground plane, and camera. 
    The innermost scope of the USD file typically defines the geometry, materials, and animation of the scene.
    """

    #import USD file
    stage = Usd.Stage.Open('dmpc-cfly.project.usd')

    """
    Prim?

    A prim is a container describing an object within the scene. Typically, a prim will contain geometry, materials, and animation.
    """

    """
    Importing a prim within the USD File
    """
    prim = stage.GetPrimAtPath('./cf2x')


    #for primes in stage if the prim contains geometry then append to string.
    mesh_prims = [x for x in stage.Traverse() if x.IsA(UsdGeom.Mesh)]
    print(mesh_prims)

    for i in range(100):

        sim_app.update()

    sim_app.close()  # cleanup application

    #save USD project
    #stage.Save()
crazyflyscene()



