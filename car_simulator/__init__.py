from gym.envs.registration import register

register(
    id='car_simulator-v0',
    entry_point='car_simulator.envs:CarSimulatorEnv',
)