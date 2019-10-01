from direct.showbase.ShowBase import ShowBase
from panda3d.core import loadPrcFileData, Vec3, WindowProperties
from panda3d.bullet import BulletWorld
from panda3d.core import Camera

from car_simulator.envs.vis_maze import generateMaze
from car_simulator.envs.simulator_setup import setLight, setBase, setCar, setCamera, getCameraImage, restartSimulation,\
    convertImageListToVideo
from car_simulator.envs.simulator_setup import initializeCarPosition, getTurnClusterPoints, isInsideTurnCluster,\
    addImageToFolder
import os
import random
import math
import numpy as np
from PIL import Image

class Simulation(ShowBase):

    def __init__(self, offScreen=False, use_multiple_cameras=False, birds_eye=False, record=False):
        if offScreen:
            loadPrcFileData("", "window-type offscreen")
        ShowBase.__init__(self)
        self.use_multiple_cameras = use_multiple_cameras
        self.birds_eye = birds_eye
        self.initializeVariables()
        self.setBackgroundColor(0.1, 0.1, 0.8, 1)
        self.gap = 4
        self.gap_timer = 10
        self.image_save_count = 0
        if self.use_multiple_cameras:
            self.win2 = self.openWindow()
            self.setBackgroundColor(0.1, 0.1, 0.8, 1, win=self.win2)
            self.win3 = self.openWindow()
            self.setBackgroundColor(0.1, 0.1, 0.8, 1, win=self.win3)
        self.setup(offScreen=offScreen)
        self.camDisplayRegion1 = self.camList[0].node().getDisplayRegion(0)
        self.imageList = []
        self.record = record
        if use_multiple_cameras:
            self.camDisplayRegion2 = self.camList[1].node().getDisplayRegion(0)
            self.camDisplayRegion3 = self.camList[2].node().getDisplayRegion(0)
        # self.my_camera.setPos(0, 0, 400)
        # self.my_camera.lookAt(0, 0, 0)
        self.taskMgr.add(self.update, 'updateWorld')


    def initializeVariables(self):
        self.asset_path = os.path.dirname(os.path.realpath(__file__))+'/../'
        self.initialImageBuffer = list()
        self.car_index_list = list()
        self.car_index_prec = list()
        self.car_rotation_list = list()
        self.direction_taken = None
        self.desired_direction = None
        self.entry_direction = None
        self.exit_direction = None

        self.collision_occurred = False
        self.do_backup = False
        self.brake = 4
        self.reverse = 8
        self.curr_time = 0
        self.prev_pos = 2, 0, 1
        self.prev_hpr = 0, 0, 0
        self.curr_image = None
        self.n_count = 0
        self.initialization_count = 10
        self.firstInitialization = True
        self.initialization = True
        self.accept('p', self.doScreenshot)

    def reinitialize_car_pos(self):
        index = random.randint(0, len(self.car_index_list)-1)
        self.carIndex = self.car_index_list[index].copy()
        self.carIndex_prec = self.car_index_list[index].copy()
        self.carRotation = self.car_rotation_list[index]
        direction_rand = random.randint(0, 1)
        if direction_rand == 0:
            self.desired_direction = [1, 0, 0]
        elif direction_rand == 1:
            self.desired_direction = [0, 0, 1]
        else:
            self.desired_direction = [0, 1, 0]

    def convertToVideo(self):
        convertImageListToVideo(self)


    def doScreenshot(self):
        self.screenshot('Collision maze.png')

    def setup(self, offScreen=False):

        #winprops = WindowProperties.size(84, 84)
        if not offScreen:
            winprops1 = WindowProperties()
            winprops1.setTitle('Front camera')
            # winprops1.set_z_order(0)
            self.win.requestProperties(winprops1)
            if self.use_multiple_cameras:
                winprops2 = WindowProperties()
                winprops2.setTitle('Left camera')
                winprops2.set_origin(0, 240)
                self.win2.requestProperties(winprops2)
                winprops3 = WindowProperties()
                winprops3.setTitle('Right camera')
                winprops3.set_origin(1500, 240)
                self.win3.requestProperties(winprops3)


        # Set lighting
        setLight(self)

        # Set World
        self.worldNP = self.render.attachNewNode('World')
        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -9.81))

        # Set plane
        setBase(self)

        # Maze
        self.mazeMatrix, self.f, self.b, self.l, self.r =\
            generateMaze(self.loader, self.asset_path, self.worldNP, self.world)
        initializeCarPosition(self)
        self.reinitialize_car_pos()

        self.tc_list = getTurnClusterPoints(self)


        # Car
        setCar(self)

        # Camera
        setCamera(self)

    def update(self, task):
        self.curr_time += 0.015
        stop_sim = False

        if self.curr_time >= 0.25:
            self.curr_time -= 0.25
            if not self.do_backup:
                if self.n_count >= self.initialization_count:
                    self.curr_image = getCameraImage(self)  # [(36, 64), (36, 64), (36, 64)]
                    stop_sim = True
                else:
                    self.n_count += 1
            else:
                self.backUp()
        if not self.initialization:
            self.engine_control()
        self.updateCarPosition()
        self.world.doPhysics(0.015, 10, 0.008)
        if self.record and self.gap_timer==0:
            self.gap_timer = self.gap
            addImageToFolder(self)
        self.gap_timer -= 1

        if not self.do_backup and self.n_count >= self.initialization_count and \
                len(self.world.contact_test(self.worldNP.find('Vehicle').node()).getContacts()) != 0:
            self.collision_occurred = True

        if stop_sim:
            self.taskMgr.stop()

        return task.again

    def updateCarPosition(self):
        vehicle_node_path = self.worldNP.find('Vehicle')
        position = vehicle_node_path.getPos()
        self.carIndex_prec[1] = (position[0] + 99) / 4
        self.carIndex_prec[0] = (99 - position[1]) / 4
        self.carIndex[1] = round(self.carIndex_prec[1])
        self.carIndex[0] = round(self.carIndex_prec[0])
        inside = isInsideTurnCluster(self)
        if not inside and self.entry_direction is not None and self.direction_taken is None:
            if (self.entry_direction + 2) % 4 == self.exit_direction:
                self.direction_taken = [0, 1, 0]
            elif (self.entry_direction + 1) % 4 == self.exit_direction:
                self.direction_taken = [0, 0, 1]
            elif (self.entry_direction + 3) % 4 == self.exit_direction:
                self.direction_taken = [1, 0, 0]
            # if (self.entry_direction + )

    def step(self, action):
        self.updateSimulationWithAction(action)
        self.taskMgr.run()# Run simulation with action
        reward = self.getReward()
        collision_occurred = self.collision_occurred
        self.collision_occurred = False
        # print(self.vehicle.getCurrentSpeedKmHour())
        return self.curr_image, reward, collision_occurred

    def getDesiredDirection(self):
        return self.desired_direction

    def updateSimulationWithAction(self, action):
        self.steering = action

        self.vehicle.setBrake(0, 2)
        self.vehicle.setBrake(0, 3)


        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

    def engine_control(self):
        vel = self.vehicle.getCurrentSpeedKmHour()
        err = 25 - vel
        engine_force = np.clip(1.25 * err * 50, -1000, 1000)
        self.vehicle.applyEngineForce(engine_force, 2)
        self.vehicle.applyEngineForce(engine_force, 3)

    def backUp(self):
        if self.reverse > 0:
            self.reverse -= 1

            self.vehicle.setBrake(0, 2)
            self.vehicle.setBrake(0, 3)
            self.vehicle.applyEngineForce(-1000, 2)
            self.vehicle.applyEngineForce(-1000, 3)
        elif self.brake > 0:
            self.brake -= 1
            self.vehicle.setSteeringValue(0, 0)
            self.vehicle.setSteeringValue(0, 1)
            self.vehicle.applyEngineForce(0, 2)
            self.vehicle.applyEngineForce(0, 3)
            self.vehicle.setBrake(100, 2)
            self.vehicle.setBrake(100, 3)
        else:
            self.do_backup = False

    def getReward(self):
        if self.collision_occurred:
            return -1
        if self.direction_taken is not None:
            self.collision_occurred = True
            if self.direction_taken == self.desired_direction:
                return 1
            else:
                return -0.5
        return 0


    def reset(self):
        if self.firstInitialization:
            self.run()
            self.firstInitialization = False
        else:
            restartSimulation(self)
        self.initialization = False
        return self.curr_image



if __name__ == '__main__':
    sim = Simulation(use_multiple_cameras=False, birds_eye=False)

    # print(collision)
    # print(dd)
    for i in range(6):
        image = sim.reset()
        print(sim.getDesiredDirection())
        for j in range(64):
            # print(j)
            # _, reward, collision = sim.step(0 if j<6 else -15)
            image, reward, collision = sim.step(0 if j< 7 else -15)
            print(reward)

            if collision:
                print(j)
                break

    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
    # sim.step(-45)
