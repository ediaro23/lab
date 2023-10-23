#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin #the pinocchio library
import numpy as np

def jointlimitscost(robot,q):
    up = max(q - robot.model.upperPositionLimit)
    down = max(robot.model.lowerPositionLimit - q)
    return max(0,max(up,down))

def jointlimitsviolated(robot,q):
    '''Return true if config not in joint limits'''
    return jointlimitscost(robot,q) > 0.

def projecttojointlimits(robot,q):
    return np.minimum(np.maximum(robot.model.lowerPositionLimit, q), robot.model.upperPositionLimit)


def collision(robot, q):
     '''Return true if in collision, false otherwise.'''
     pin.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data,q)
     # if pin.computeCollisions(robot.collision_model,robot.collision_data,False):
     #     for k in range(len(robot.collision_model.collisionPairs)): 
     #         cr = robot.collision_data.collisionResults[k]
     #         cp = robot.collision_model.collisionPairs[k]
     #         if cr.isCollision():
     #             print("collision pair:",robot.collision_model.geometryObjects[cp.first].name,",",robot.collision_model.geometryObjects[cp.second].name,"- collision:","Yes" if cr.isCollision() else "No")
     
     return pin.computeCollisions(robot.collision_model,robot.collision_data,False)
    
def distanceToObstacle(robot, q):
      '''Return the shortest distance between robot and the obstacle. '''
      geomidobs = robot.collision_model.getGeometryId('obstaclebase_0')
      geomidtable = robot.collision_model.getGeometryId('baseLink_0')
      pairs = [i for i, pair in enumerate(robot.collision_model.collisionPairs) if pair.second == geomidobs or pair.second == geomidtable]
      pin.framesForwardKinematics(robot.model,robot.data,q)
      pin.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data,q)
      dists = [pin.computeDistance(robot.collision_model, robot.collision_data, idx).min_distance for idx in pairs]      
      
      # pairsId = [pair.first for i, pair in enumerate(robot.collision_model.collisionPairs) if pair.second == geomidobs or pair.second == geomidtable]
      # names = [robot.collision_model.geometryObjects[idx].name for idx in pairsId ]
      # for name, dist in zip(names,dists):
      #     print ("name / distance ", name, " / ", dist)
      # print(min (dists))
      return min(dists)

    
def getcubeplacement(cube, hookname = None):
    oMf = cube.collision_model.geometryObjects[0].placement
    if hookname is not None:
        frameid = cube.model.getFrameId(hookname)
        oMf *= cube.data.oMf[frameid] 
    return oMf
        

def setcubeplacement(robot, cube, oMf):
    q = cube.q0
    robot.visual_model.geometryObjects[-1].placement = oMf
    robot.collision_model.geometryObjects[-1].placement = oMf
    cube.visual_model.geometryObjects[-1].placement = oMf
    cube.collision_model.geometryObjects[0].placement = oMf    
    pin.updateGeometryPlacements(cube.model,cube.data,cube.collision_model,cube.collision_data,q)
    

    
from setup_pinocchio import setuppinocchio
from setup_meshcat import setupmeshcat     
from config import MESHCAT_URL

def setupwithmeshcat(url=MESHCAT_URL):
     '''setups everything to work with the robot and meshcat'''
     robot, table, obstacle, cube = setuppinocchio()
     viz = setupmeshcat(robot)
     return robot, cube, viz
 
from setup_pybullet import setuppybullet
def setupwithpybullet():
     '''setups everything to work with the robot and pybullet'''
     robot, table, obstacle, cube = setuppinocchio()   
     sim = setuppybullet(robot)
     sim.setTorqueControlMode()
     return robot, sim, cube
 
    
def setupwithpybulletandmeshcat(url=MESHCAT_URL):
     '''setups everything to work with the robot, pybullet AND meshcat'''
     robot, table, obstacle, cube = setuppinocchio() 
     viz = setupmeshcat(robot)
     sim = setuppybullet(robot)
     sim.setTorqueControlMode()
     return robot, sim, cube, viz
 

import time
   
def rununtil(f, t, *args, **kwargs):
    '''starts a timer, runs a function f then waits until t seconds have passed since timer started'''
    t = time.perf_counter()
    # Call the provided function f2 with its arguments
    result = f(*args, **kwargs)
    t+=0.001
    while(time.perf_counter()-t<0.001):
        time.sleep(0.0001) # Weird loop for parallel execution (should not be needed in this project)
    return result