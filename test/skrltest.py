# import the environment loader
from skrl.envs.torch import load_omniverse_isaacgym_env
from skrl.envs.torch import wrap_env
from skrl.utils.omniverse_isaacgym_utils import get_env_instance
from omniisaacgymenvs.utils.config_utils.sim_config import SimConfig


print("INITALIZING SCRIPT")
# load environment
env = load_omniverse_isaacgym_env(task_name="Crazyflie")
env = wrap_env(env, wrapper="omniverse-isaacgym")
obs, info = env.reset()

print(info)
terminated, truncated = False, False
print("================")
print(f"DONE LOADING ENV x0 {obs}")
print("================")
while not (terminated or truncated):
    env.render()
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
env.close()
