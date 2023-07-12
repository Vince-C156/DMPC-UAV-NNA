# DMPC-UAV-NNA
Dynamic Model Predictive Controller, a Unified approach via Neural Network Augmentation.

Model Predictive Control, or MPC, typically use a simplified linear discrete time derivation of real world non-linear continous dynamics. As a result a change in model dynamics or harsh real world disturbances could lead to complete failure of these systems. 

We try utilize the strengths and compensate the weaknesses of Optimal Control and Reinforcement Learning with a combined control policy. Our policy aims to leverage the convergence guarentees in MPC formulations and the generalization capabilities in Policy Gradient Optimization, PPO, by having our PPO policy predict model parameters and their uncertanities at runtime to minimize model error in scenerios with changing dynamics.

Python3.10, Cuda-12.0, Ubuntu 20.04, Python3.7(Isaac Python runtime)

Libraries: IsaacSim, OmniisaacGymEnvs, skrl, matplotlib, numpy, px4, ros
