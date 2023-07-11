from pxr import Usd, UsdGeom
from omni.isaac.kit import SimulationApp
from omni.isaac.core.articulations import Articulation, ArticulationGripper, ArticulationSubset, ArticulationView
from omni.isaac.core.controllers import ArticulationController, BaseController, BaseGripperController
from omni.isaac.core.loggers import DataLogger
from omni.isaac.core.materials import OmniGlass, OmniPBR, ParticleMaterial, ParticleMaterialView, PhysicsMaterial, PreviewSurface, VisualMaterial
from omni.isaac.core.objects import DynamicCapsule, DynamicCone, DynamicCuboid, DynamicCylinder, DynamicSphere
from omni.isaac.core.objects import FixedCapsule, FixedCone, FixedCuboid, FixedCylinder, FixedSphere, GroundPlane
from omni.isaac.core.objects import VisualCapsule, VisualCone, VisualCuboid, VisualCylinder, VisualSphere
from omni.isaac.core.physics_context import PhysicsContext
from omni.isaac.core.prims import BaseSensor, ClothPrim, GeometryPrim, ParticleSystem, RigidPrim, XFormPrim
from omni.isaac.core.prims import ClothPrimView, GeometryPrimView, ParticleSystemView, RigidContactView, RigidPrimView, XFormPrimView
from omni.isaac.core.robots import Robot, RobotView
from omni.isaac.core.scenes import Scene, SceneRegistry
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.world import World
from omni.isaac.core.tasks import BaseTask, FollowTarget, PickPlace, Stacking


config = {"headless": False,
          "renderer": "RayTracedLighting"}

# Any Omniverse level imports must occur after the class is instantiated. 
# APIs are provided by the extension/runtime plugin system, 
# it must be loaded before they will be available to import.
simulation_app = SimulationApp(config)
"""
What is USD? (Universal Scene Description)

open and extensible framework and ecosystem for describing, composing, simulating and collaborating within 3D worlds,
originally developed by Pixar Animation Studios. USD is more than a file format, it’s an open source 3D scene description used for 3D content creation and interchange among different tools.
As a result of its power and versatility, it’s being widely adopted, not only in the visual effects community, but also in architecture, design, robotics, manufacturing, and other industries.


USD, can be seen as a nested file type that can contain other file types, such as OBJ, Alembic, FBX, and USD itself. Generally the outermost scope of the USD file typically defines characteristics of a scene such as the lighting, ground plane, and camera. 
The innermost scope of the USD file typically defines the geometry, materials, and animation of the scene.
"""

#import USD file
stage = Usd.Stage.Open('/home/vince/Documents/crazyflyuavf.project.usd')

"""
Prim?

A prim is a container describing an object within the scene. Typically, a prim will contain geometry, materials, and animation.
"""

"""
Importing a prim within the USD File
"""
prim = stage.GetPrimAtPath('./Crazyflie')


#for primes in stage if the prim contains geometry then append to string.
mesh_prims = [x for x in stage.Traverse() if x.IsA(UsdGeom.Mesh)]
print(mesh_prims)

for i in range(100):

    simulation_app.update()

simulation_app.close()  # cleanup application

#save USD project
#stage.Save()

