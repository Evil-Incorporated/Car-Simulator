from gym.envs.registration import register

register(
    id='Car-Simulator-v0',
    entry_point='car_simulator.envs:Simulator',
)