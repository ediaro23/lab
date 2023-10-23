import subprocess
import math
import time
import sys
import os
import numpy as np
import pybullet as pyb
import pybullet_data

# setup paths and load the core
abs_path = os.path.dirname(os.path.realpath(__file__))
root_path = abs_path
core_path = root_path + '/core'
sys.path.append(core_path)
from Pybullet_Simulation_base import Simulation_base

from config import NEXTAGE_URDF, USE_PYBULLET_GUI, TABLE_URDF,  OBSTACLE_URDF, CUBE_URDF

from config import ROBOT_PLACEMENT, TABLE_PLACEMENT, OBSTACLE_PLACEMENT, CUBE_PLACEMENT
from config import DT
import pinocchio as pin



pybulletConfigs = {
    "simulation": pyb,
    "pybullet_extra_data": pybullet_data,
    "gui": USE_PYBULLET_GUI,
    "panels": False,
    "realTime": False,
    "controlFrequency": 1000,
    "updateFrequency": 250,
    "gravity": -9.81,
    "floor": True,
    "cameraSettings": (1.2, 90, -22.8, (-0.12, -0.01, 0.99))
}

robotConfigs = {
    "robotPath": NEXTAGE_URDF,
    "robotPIDConfigs": core_path + "/PD_gains.yaml",
    "robotStartPos": ROBOT_PLACEMENT.translation,  # (x, y, z)
    "robotStartOrientation": [0, 0, 0, 1],  # (x, y, z, w)
    "fixedBase": False,        # True | False
    "colored": False          # True | False
}

def load_object(sim,urdf,placement,isFixed):
    placementquat = pin.SE3ToXYZQUAT(placement)
    return sim.p.loadURDF(
        fileName              = urdf, 
        basePosition          = placementquat[:3],            
        baseOrientation       = placementquat[-4:],                                  
        useFixedBase          = isFixed,             
        globalScaling         = 1.
    )

class Simulation(Simulation_base):
    """A Bullet simulation involving Nextage robot"""

    def __init__(self, pinocchiorobot):
        """Constructor
        Creates a simulation instance with Nextage robot.
        For the keyword arguments, please see in the Pybullet_Simulation_base.py
        """
        super().__init__(pybulletConfigs, robotConfigs)        
        self.readyForSimu = False
        self.tableId = load_object(self,TABLE_URDF, TABLE_PLACEMENT, True)
        self.cubeId = load_object(self,CUBE_URDF, CUBE_PLACEMENT, False)
        self.obstacleId = load_object(self,OBSTACLE_URDF, OBSTACLE_PLACEMENT, True)  
        self.linkpinocchioandbullet(pinocchiorobot)

    
    def linkpinocchioandbullet(self, pinocchiorobot):
        """maps pinocchio joint ids with pybullet
        """
        self.bullet_names2indices = {
            pyb.getJointInfo(1, i)[1].decode(): i for i in range(pyb.getNumJoints(1))
        }
        self.bulletCtrlJointsInPinOrder = [
            self.bullet_names2indices[n] for n in pinocchiorobot.model.names[1:]
        ]

    def getpybulletstate(self):
        """gets the current q and vq
        """
        # Get articulated joint pos and vel
        xbullet = pyb.getJointStates(self.robot, self.bulletCtrlJointsInPinOrder)
        q = np.array([x[0] for x in xbullet])
        vq = np.array([x[1] for x in xbullet])
        return q, vq
    

    def setqsim(self, q):
        """sets the current q
        """
        for i, qi in zip(self.bulletCtrlJointsInPinOrder, q):
            self.p.resetJointState(bodyUniqueId=1, jointIndex=i, targetValue=qi)

    def setTorqueControlMode(self):
        """
        Disable default position controler in torque controlled joints
        Default controller will take care of other joints
        """
        pyb.setJointMotorControlArray(
            self.robot,
            jointIndices=self.bulletCtrlJointsInPinOrder,
            controlMode=pyb.VELOCITY_CONTROL,
            forces=[0.0 for _ in self.bulletCtrlJointsInPinOrder],
        )
        self.readyForSimu = True
        
    def step(self, torques = None):
        """
        Advances the simulation by DT, inputing the torques passed as parameters
        """
        assert self.readyForSimu
        if torques is None:
            torques = [0.0 for _ in self.bulletCtrlJointsInPinOrder]
        pyb.setJointMotorControlArray(
            self.robot,
            self.bulletCtrlJointsInPinOrder,
            controlMode=pyb.TORQUE_CONTROL,
            forces=torques,
        )
        pyb.stepSimulation()

        
    

def setuppybullet(pinocchiorobot):
    sim = Simulation(pinocchiorobot)    
    pyb.setTimeStep(DT)
    return sim


################################################################################
################################################################################





if __name__ == "__main__":
    
    from tools import setupwithmeshcat
    robot, cube, viz = setupwithmeshcat()
    sim = setuppybullet(robot)
    
    sim.setTorqueControlMode()
    
    # #set pin config to pybullet
    # q = pin.randomConfiguration(robot.model)
    # viz.display(q)
    # setqsim(sim,q)
    
    # qsim, vqsim = getpybulletstate(sim)
    # print(np.linalg.norm(q - qsim))
    