#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin #the pinocchio library
import numpy as np

def jointlimitscost(robot,q):
    '''Return the maximum joint violation of a configuration'''
    up = max(q - robot.model.upperPositionLimit)
    down = max(robot.model.lowerPositionLimit - q)
    return max(0,max(up,down))

def jointlimitsviolated(robot,q):
    '''Return true if config not in joint limits'''
    return jointlimitscost(robot,q) > 0.

def projecttojointlimits(robot,q):
    '''Projects a configuration to the closest one that satisfies joint limits'''
    return np.minimum(np.maximum(robot.model.lowerPositionLimit, q), robot.model.upperPositionLimit)


def collision(robot, q):
     '''Return true if in collision, false otherwise.'''
     pin.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data,q)
     return pin.computeCollisions(robot.collision_model,robot.collision_data,False)
    
def distanceToObstacle(robot, q):
      '''Return the shortest distance between robot and the obstacle. '''
      geomidobs = robot.collision_model.getGeometryId('obstaclebase_0')
      geomidtable = robot.collision_model.getGeometryId('baseLink_0')
      pairs = [i for i, pair in enumerate(robot.collision_model.collisionPairs) if pair.second == geomidobs or pair.second == geomidtable]
      pin.framesForwardKinematics(robot.model,robot.data,q)
      pin.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data,q)
      dists = [pin.computeDistance(robot.collision_model, robot.collision_data, idx).min_distance for idx in pairs]  
      return min(dists)


def collisionDistance(robot, cube, position):
     '''Return the minimal distance between cube and environment. '''
     oMcurrent = getcubeplacement(cube)
     oMf = pin.SE3(oMcurrent.rotation,position)
     setcubeplacement(robot, cube, oMf)
     if pin.computeCollisions(cube.collision_model,cube.collision_data,False):
         return +0.00001
     idx = pin.computeDistances(cube.collision_model,cube.collision_data)
     return -cube.collision_data.distanceResults[idx].min_distance +0.00001

def getcubeplacement(cube, hookname = None):
    '''gets the cube placement in the world frame. if hookname is provided, returns placement of the hook instead'''
    oMf = cube.collision_model.geometryObjects[0].placement
    if hookname is not None:
        frameid = cube.model.getFrameId(hookname)
        oMf *= cube.data.oMf[frameid] 
    return oMf
        

def setcubeplacement(robot, cube, oMf):
    '''sets the cube position to the provided placement in world frame'''
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
     '''setup everything to work with the robot and meshcat'''
     robot, table, obstacle, cube = setuppinocchio()
     viz = setupmeshcat(robot, url)
     return robot, cube, viz
 
    
