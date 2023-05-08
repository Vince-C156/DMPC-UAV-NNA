import numpy as np
import pydot
from IPython.display import HTML, SVG, display
from underactuated.scenarios import AddFloatingRpyJoint
from pydrake.all import ( 
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    GenerateHtml,
    InverseDynamicsController,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    MultibodyPlant,
    Parser,
    Simulator,
    StartMeshcat,
)

#start meshcat

meshcat = StartMeshcat()

#context = plant.CreateDefaultContext()
#print(context)

#create diagram builder (starts up all block stuff)
meshcat.DeleteAddedControls() #? resets meshcat
builder = DiagramBuilder()



# Adds both MultibodyPlant and the SceneGraph, and wires them together.
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
# Note that we parse into both the plant and the scene_graph here.

(model_instance,)=Parser(plant, scene_graph).AddModelsFromUrl(
        "package://drake/examples/quadrotor/quadrotor.urdf"
)
"""
AddFloatingRpyJoint(
    plant,
    plant.GetFrameByName("base_link"),
    model_instance,
    use_ball_rpy=False,
)
"""

plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))
plant.Finalize()
"""
body_index = plant.GetBodyByName("base_link").index()
# Default parameters from quadrotor_plant.cc:
L = 0.15  # Length of the arms (m).
kF = 1.0  # Force input constant.
kM = 0.0245  # Moment input constant.

# Note: Rotors 0 and 2 rotate one way and rotors 1 and 3 rotate the other.
prop_info = [
    PropellerInfo(body_index, RigidTransform([L, 0, 0]), kF, kM),
    PropellerInfo(body_index, RigidTransform([0, L, 0]), kF, -kM),
    PropellerInfo(body_index, RigidTransform([-L, 0, 0]), kF, kM),
    PropellerInfo(body_index, RigidTransform([0, -L, 0]), kF, -kM),
]
propellers = builder.AddSystem(Propeller(prop_info))
    builder.Connect(
    propellers.get_output_port(),
    plant.get_applied_spatial_force_input_port(),
)
builder.Connect(
    plant.get_body_poses_output_port(),
    propellers.get_body_poses_input_port(),
)
builder.ExportInput(propellers.get_command_input_port(), "u")


"""
"""
Parser(plant, scene_graph).AddModelsFromUrl(
    "package://drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"
)
"""
#attaches multibody plant to world frame (would fall through environment ground)
#plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))
#plant.Finalize()

#plant.get_actuation_input_port().FixValue(plant_context, np.zeros(7))

# Adds the MeshcatVisualizer and wires it to the SceneGraph.
visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

#print(context)
diagram = builder.Build()
context = diagram.CreateDefaultContext()
plant_context = plant.GetMyMutableContextFromRoot(context)
#plant.get_actuation_input_port().FixValue(plant_context, np.zeros(4))
#diagram = builder.Build()

#SVG(pydot.graph_from_dot_data(diagram.GetGraphvizString())[0].create_svg())

diagram.set_name("diagram")

HTML(
    '<script src="https://unpkg.com/gojs/release/go.js"></script>'
    + GenerateHtml(diagram)
)

print(context)
simulator = Simulator(diagram, context)
simulator.set_target_realtime_rate(1.0)
#context = diagram.CreateDefaultContext()
#diagram.ForcedPublish(context)
simulator.AdvanceTo(5.0)

input()
