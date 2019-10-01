import gym
from gym import error, spaces, utils
from gym.utils import seeding
from car_simulator.envs.simulation import Simulation



class SimulatorGym(gym.Env):
    metadata = {'render.modes':['human']}

    def __init__(self, offScreen=False):
        self.car = Simulation(offScreen=offScreen)

    def step(self, action):
        image, reward, collision_occured = self.car.step(action=action)
        return image, reward, collision_occured

    def reset(self):
        return self.car.reset()

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    # def render(self, mode='human'):
    #     for i in range(3):
    #         for j in range(3):
    #             print(self.state[i][j], end=" ")
    #         print("")