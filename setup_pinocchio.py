#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  4 11:09:02 2023

@author: stonneau
"""

import pinocchio as pin #the pinocchio library
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np 

## ROBOT INIT

#Where to find the robot model and environment
from config import NEXTAGE_URDF, MESH_DIR, NEXTAGE_SRDF, ROBOT_PLACEMENT, TABLE_URDF, TABLE_MESH, TABLE_PLACEMENT, OBSTACLE_URDF, OBSTACLE_MESH, OBSTACLE_PLACEMENT, CUBE_URDF    ,CUBE_MESH    , CUBE_PLACEMENT


def loadrobot():    
    return RobotWrapper.BuildFromURDF(NEXTAGE_URDF, MESH_DIR) 
    
def translate(robotwrapper, oMf):
    for g in robotwrapper.collision_model.geometryObjects:
            g.placement= oMf * g.placement
    for g in robotwrapper.visual_model.geometryObjects:
            g.placement=  oMf * g.placement
    
def translaterobot(robotwrapper, oMf):
    for i in range(2):
        robotwrapper.collision_model.geometryObjects[i].placement = oMf * robotwrapper.collision_model.geometryObjects[i].placement 
        robotwrapper.visual_model.geometryObjects[i].placement  = oMf * robotwrapper.visual_model.geometryObjects[i].placement 
    robotwrapper.model.jointPlacements[1] =  oMf * robotwrapper.model.jointPlacements[1]
              

def addcollisiontorobot(robot, objectenv):
    #change joint parent
    #geomdata.omg, data.omi*geomedel.placement / omf #joint joint id > 0
    for obs in objectenv.collision_model.geometryObjects:
        robot.collision_model.addGeometryObject(obs)  # Add object to collision model
    for obs in objectenv.visual_model.geometryObjects:
        robot.visual_model.addGeometryObject(obs)  # Add object to collision model
                
    
def loadobject(parentrobot, urdf,mesh, oMf):
    robot = RobotWrapper.BuildFromURDF(urdf, mesh) 
    robot.collision_data = robot.collision_model.createData()
    translate(robot   , oMf)
    addcollisiontorobot(parentrobot,robot)    
    pin.framesForwardKinematics(robot.model,robot.data,robot.q0)
    return robot


def finalisecollisionsetup(robot):
    robot.collision_data = robot.collision_model.createData()
    robot.collision_model.addAllCollisionPairs()    
    # Remove collision pairs per SRDF
    pin.removeCollisionPairs(robot.model, robot.collision_model, NEXTAGE_SRDF, False)
    robot.collision_data = pin.GeometryData(robot.collision_model)
    robot.visual_data    = pin.GeometryData(robot.visual_model   )
        
def addcubecollision(cube, obstacle):
    for obs in obstacle.collision_model.geometryObjects:
        cube.collision_model.addGeometryObject(obs)  # Add object to collision model
    
def finalisecubecollisionsetup(cube):
    cube.collision_model.addAllCollisionPairs()   
    cube.collision_model.addCollisionPair(pin.CollisionPair(0,1))
    cube.collision_model.addCollisionPair(pin.CollisionPair(0,2))
    cube.collision_data = cube.collision_model.createData()
    

def setuppinocchio():
    robot = RobotWrapper.BuildFromURDF(NEXTAGE_URDF, MESH_DIR) 
    translaterobot(robot, ROBOT_PLACEMENT)
    table    = loadobject(robot, TABLE_URDF   ,TABLE_MESH   , TABLE_PLACEMENT)
    obstacle = loadobject(robot, OBSTACLE_URDF,OBSTACLE_MESH, OBSTACLE_PLACEMENT)
    cube     = loadobject(robot, CUBE_URDF    ,CUBE_MESH    , CUBE_PLACEMENT)
    addcubecollision(cube, table)
    addcubecollision(cube, obstacle)
    finalisecollisionsetup(robot)
    finalisecubecollisionsetup(cube)
    return robot, table, obstacle, cube
