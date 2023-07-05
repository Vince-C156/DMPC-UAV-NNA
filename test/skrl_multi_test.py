import threading

print("Pyisaac skrl_multi_test.py task=[taskname]")
# import the environment loader
from skrl.envs.torch import load_omniverse_isaacgym_env

# load environment
env = load_omniverse_isaacgym_env(multi_threaded=True, timeout=30)

...

# start training in a separate thread
threading.Thread(target=trainer.train).start()

# run the simulation in the main thread
env.run()