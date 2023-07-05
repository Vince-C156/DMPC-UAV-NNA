# import the environment loader
from skrl.envs.torch import load_omniverse_isaacgym_env

# load environment
env = load_omniverse_isaacgym_env(task_name="Crazyflie")
