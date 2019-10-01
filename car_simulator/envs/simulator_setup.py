from panda3d.core import AmbientLight, Vec3, Vec4, DirectionalLight, BitMask32, Point3, TransformState, Camera
from panda3d.bullet import BulletPlaneShape, BulletRigidBodyNode, BulletBoxShape, BulletVehicle, ZUp
import numpy
import random
import sys
import time
from PIL import Image
import cv2
import scipy.misc
import os

shape = BulletBoxShape(Vec3(2, 2, 2))

def setLight(environment):
    alight = AmbientLight('ambientLight')
    alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
    alightNP = environment.render.attachNewNode(alight)

    dlight = DirectionalLight('directionalLight')
    dlight.setDirection(Vec3(1, 1, -1))
    dlight.setColor(Vec4(0.7, 0.7, 0.7, 1))
    dlightNP = environment.render.attachNewNode(dlight)

    environment.render.clearLight()
    environment.render.setLight(alightNP)
    environment.render.setLight(dlightNP)


def setBase(environment):
    shape = BulletPlaneShape(Vec3(0, 0, 1), 0)

    np = environment.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
    np.node().addShape(shape)
    np.setPos(0, 0, -1)
    np.setCollideMask(BitMask32.allOn())

    environment.world.attachRigidBody(np.node())

    environment.baseNP = environment.loader.loadModel(environment.asset_path + 'assets/base2.egg')
    environment.baseNP.setScale(12.75)
    environment.baseNP.setTwoSided(True)
    environment.baseNP.reparentTo(np)
    # environment.loader.loadModel(os.path.dirname(os.path.realpath(__file__))+'/../assets/base2.egg')

def setCar(environment):
    # Chassis
    shape = BulletBoxShape(Vec3(0.6, 1.4, 0.5))
    ts = TransformState.makePos(Point3(0, 0, 0.5))

    np = environment.worldNP.attachNewNode(BulletRigidBodyNode('Vehicle'))
    np.node().addShape(shape, ts)
    np.setPos(-99 +environment.carIndex[1]*4, -environment.carIndex[0]*4 + 99, 1)
    np.setHpr(environment.carRotation, 0, 0)
    np.node().setMass(50.0)
    np.node().setLinearVelocity(Vec3(0, 0, 0))
    np.node().setDeactivationEnabled(False)
    environment.world.attachRigidBody(np.node())

    # Vehicle
    environment.vehicle = BulletVehicle(environment.world, np.node())
    environment.vehicle.setCoordinateSystem(ZUp)
    environment.world.attachVehicle(environment.vehicle)

    environment.yugoNP = environment.loader.loadModel(environment.asset_path + 'assets/yugo.egg')
    environment.yugoNP.reparentTo(np)
    # environment.yugoNP.setTwoSided(True)

    # Right front wheel
    np = environment.loader.loadModel(environment.asset_path + 'assets/yugotireR.egg')
    np.reparentTo(environment.worldNP)
    addWheel(environment, Point3(0.70, 1.05, 0.3), True, np)

    # Left front wheel
    np = environment.loader.loadModel(environment.asset_path + 'assets/yugotireL.egg')
    np.reparentTo(environment.worldNP)
    addWheel(environment, Point3(-0.70, 1.05, 0.3), True, np)

    # Right rear wheel
    np = environment.loader.loadModel(environment.asset_path + 'assets/yugotireR.egg')
    np.reparentTo(environment.worldNP)
    addWheel(environment, Point3(0.70, -1.05, 0.3), False, np)

    # Left rear wheel
    np = environment.loader.loadModel(environment.asset_path + 'assets/yugotireL.egg')
    np.reparentTo(environment.worldNP)
    addWheel(environment, Point3(-0.70, -1.05, 0.3), False, np)

    # Steering info
    environment.steering = 0.0  # degree
    environment.steeringClamp = 45.0  # degree
    environment.steeringIncrement = 120.0  # degree per second

def addWheel(environment, pos, front, np):
    wheel = environment.vehicle.createWheel()

    wheel.setNode(np.node())
    wheel.setChassisConnectionPointCs(pos)
    wheel.setFrontWheel(front)

    wheel.setWheelDirectionCs(Vec3(0, 0, -1))
    wheel.setWheelAxleCs(Vec3(1, 0, 0))
    wheel.setWheelRadius(0.25)
    wheel.setMaxSuspensionTravelCm(40.0)

    wheel.setSuspensionStiffness(40.0)
    wheel.setWheelsDampingRelaxation(2.3)
    wheel.setWheelsDampingCompression(4.4)
    wheel.setFrictionSlip(100.0)
    wheel.setRollInfluence(0.1)

def setCamera(environment):
    if not environment.birds_eye:
        environment.cam.reparentTo(environment.worldNP.find('Vehicle'))
        environment.cam.setPos(0, 0, 1.75)
        environment.cam.lookAt(0, 2, 1.5)
    elif environment.birds_eye:
        environment.cam.setPos(0, 0, 300)
        environment.cam.lookAt(0, 0, 0)
    if environment.use_multiple_cameras:
        environment.l_cam = environment.camList[1]
        # environment.l_cam = environment.render.attachNewNode(environment.left_camera)
        environment.l_cam.reparentTo(environment.worldNP.find('Vehicle'))
        environment.l_cam.setPos(0, 0, 1.75)
        environment.l_cam.lookAt(0, 2, 1.5)
        environment.l_cam.setHpr(30, 0, 0)
        environment.r_cam = environment.camList[2]
        environment.r_cam.reparentTo(environment.worldNP.find('Vehicle'))
        environment.r_cam.setPos(0, 0, 1.75)
        environment.r_cam.lookAt(0, 2, 1.5)
        environment.r_cam.setHpr(-30, 0, 0)

def getCameraImage(environment):
    if not environment.use_multiple_cameras:
        return getImageFromCurrentCamera(environment)
    else:
        image_list = []
        for i in range(3):
            image_list.append(getImageFromCurrentCamera(environment, i))
        return image_list


def addImageToFolder(environment):
    tex = environment.camDisplayRegion1.getScreenshot()
    if tex is not None:
        image = numpy.frombuffer(tex.getRamImageAs('RGBA'), numpy.uint8)
        image.shape = (tex.getYSize(), tex.getXSize(), tex.getNumComponents())
        image = numpy.flipud(image)
        image = image[..., :-1]
        image = cv2.resize(image, (64, 36), interpolation=cv2.INTER_AREA)
        scipy.misc.imsave('Images/outfile' + str(environment.image_save_count + 1) + '.jpg', image)
        environment.image_save_count += 1

def convertImageListToVideo(environment=None):

    images = []
    for i in range(421):
        images.append('outfile' + str(i + 1) + '.jpg')

    image_folder = 'Images'
    video_name = 'video.avi'

    print(images)
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    frame = cv2.resize(frame, (128, 72), interpolation=cv2.INTER_AREA)
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 20, (width, height))

    for image in images:
        image_frame = cv2.imread(os.path.join(image_folder, image))
        image_frame = cv2.resize(image_frame, (128, 72), interpolation=cv2.INTER_AREA)
        video.write(image_frame)

    cv2.destroyAllWindows()
    video.release()

def getImageFromCurrentCamera(environment, index=None):
    tex = None
    if index is None or index == 0:
        tex = environment.camDisplayRegion1.getScreenshot()
    elif index == 1:
        tex = environment.camDisplayRegion2.getScreenshot()
    elif index == 2:
        tex = environment.camDisplayRegion3.getScreenshot()
    if tex is not None:
        image = numpy.frombuffer(tex.getRamImageAs('RGBA'), numpy.uint8)
        image.shape = (tex.getYSize(), tex.getXSize(), tex.getNumComponents())
        image = numpy.flipud(image)
        image = image[..., :-1]
        image = 0.2989 * image[:, :, 0] + 0.5870 * image[:, :, 1] + 0.1140 * image[:, :, 2]
        image = cv2.resize(image, (64 , 36 ), interpolation=cv2.INTER_AREA)
        # img = Image.fromarray(image)
        return numpy.asarray(image, dtype=numpy.float32)
    return None


def initializeCarPosition(environment):
    f = environment.f
    b = environment.b
    l = environment.l
    r = environment.r
    mazeMatrix = environment.mazeMatrix
    for i in range(mazeMatrix.shape[0]):
        for j in range(mazeMatrix.shape[1]):
            if mazeMatrix[i, j] == f or mazeMatrix[i, j] == b:
                if j!=0 and mazeMatrix[i, j-1] == mazeMatrix[i, j]:
                    environment.car_index_list[-1][1] += 0.5
                else:
                    environment.car_index_list.append([i, j])
                    environment.car_rotation_list.append(0 if mazeMatrix[i, j] == f else 180 )

    for j in range(mazeMatrix.shape[1]):
        for i in range(mazeMatrix.shape[0]):
            if mazeMatrix[i, j] == l or mazeMatrix[i, j] == r:
                if i!=0 and mazeMatrix[i-1, j] == mazeMatrix[i, j]:
                    environment.car_index_list[-1][0] += 0.5
                else:
                    environment.car_index_list.append([i,j])
                    environment.car_rotation_list.append(90 if mazeMatrix[i, j]==l else 270 )

def getTurnClusterPoints(environment):
    mazeMatrix = environment.mazeMatrix
    tc_list = []
    for i in range(mazeMatrix.shape[0] - 3):
        for j in range(mazeMatrix.shape[1] - 3):
            if mazeMatrix[i,j] == 1 and mazeMatrix[i,j+3] == 1 and mazeMatrix[i+3,j] == 1 and mazeMatrix[i+3,j+3] == 1:
                if mazeMatrix[i+1, j] == 0 and mazeMatrix[i+2, j] == 0:
                    if mazeMatrix[i, j+1] == 0 and mazeMatrix[i, j+2] == 0:
                        if mazeMatrix[i+3, j+1] == 0 and mazeMatrix[i+3, j+2] == 0:
                            if mazeMatrix[i+1, j+3] == 0 and mazeMatrix[i+2, j+3] == 0:
                                tc_list.append([i, j])
    return tc_list

def isInsideTurnCluster(environment):
    # print(environment.carIndex, environment.tc_list[0])
    for point in environment.tc_list:
        if environment.carIndex[0] >= point[0] and environment.carIndex[0] <= point[0] + 3:
            if environment.carIndex[1] >= point[1] and environment.carIndex[1] <= point[1] + 3:
                if environment.entry_direction is None:
                    if environment.carIndex[0] == point[0]:
                        environment.entry_direction = 0
                    elif environment.carIndex[1] == point[1]:
                        environment.entry_direction = 1
                    elif environment.carIndex[0] == point[0] + 3:
                        environment.entry_direction = 2
                    else:
                        environment.entry_direction = 3
                else:
                    if environment.carIndex[0] == point[0]:
                        environment.exit_direction = 0
                    elif environment.carIndex[1] == point[1]:
                        environment.exit_direction = 1
                    elif environment.carIndex[0] == point[0] + 3:
                        environment.exit_direction = 2
                    else:
                        environment.exit_direction = 3
                return True

    return False



def restartSimulation(environment):
    environment.reinitialize_car_pos()
    environment.inside_cluster = False
    vehicle_node_path = environment.worldNP.find('Vehicle')
    vehicle_node_path.setPos(-99 + environment.carIndex[1] * 4, -environment.carIndex[0] * 4 + 99, 1)
    vehicle_node_path.setHpr(environment.carRotation, 0, 0)
    vehicle_node_path.node().clearForces()
    vehicle_node_path.node().setLinearVelocity(Vec3(0, 0, 0))
    vehicle_node_path.node().setAngularVelocity(Vec3(0, 0, 0))
    vehicle_node_path.node().clearForces()
    environment.steering = 0
    environment.vehicle.setSteeringValue(environment.steering, 0)
    environment.vehicle.setSteeringValue(environment.steering, 1)
    environment.vehicle.applyEngineForce(0, 2)
    environment.vehicle.applyEngineForce(0, 3)
    environment.vehicle.setBrake(0, 2)
    environment.vehicle.setBrake(0, 3)
    # environment.vehicle.setSteeringValue(environment.steering, 0)
    # environment.vehicle.setSteeringValue(environment.steering, 1)
    environment.collision_occurred = False
    environment.initialization = True
    environment.n_count = 0
    environment.brake = 4
    environment.reverse = 7

    environment.entry_direction = None
    environment.exit_direction = None
    environment.direction_taken = None
    # environment.desired_direction = None
    # environment.do_backup = True

    environment.taskMgr.run()

if __name__ == '__main__':
    convertImageListToVideo()