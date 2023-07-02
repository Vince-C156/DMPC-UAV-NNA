# import the environment loader
from skrl.envs.torch import load_omniverse_isaacgym_env
from skrl.envs.torch import wrap_env

print("INITALIZING SCRIPT")
# load environment
env = load_omniverse_isaacgym_env(task_name="Crazyflie")
#env = wrap_env(env)
obs = env.reset()
terminated, truncated = False, False
print("================")
print(f"DONE LOADING ENV x0 {obs}")
print("================")
while not (terminated or truncated):
    env.render()
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
env.close()
