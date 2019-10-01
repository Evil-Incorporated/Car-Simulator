import gym
from gym import error, spaces, utils
from gym.utils import seeding
from panda3d.bullet import BulletWorld


class Simulator(gym.Env):
    metadata = {'render.modes':['human']}

    def __init__(self):
        pass

    def step(self, target):
        pass

    def reset(self):
        pass

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    # def render(self, mode='human'):
    #     for i in range(3):
    #         for j in range(3):
    #             print(self.state[i][j], end=" ")
    #         print("")