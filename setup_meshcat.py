#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  4 11:09:02 2023

@author: stonneau
"""

from utils.meshcat_viewer_wrapper import MeshcatVisualizer # the meshcat visualiser
import sys
from config import USE_MESHCAT, MESHCAT_URL, LEFT_HAND, RIGHT_HAND, CUBE_PLACEMENT_TARGET
from tools import getcubeplacement


FRAME_NAMES =  [LEFT_HAND+"frame",RIGHT_HAND+"frame", ]
FRAME_JOINT_NAMES =  [LEFT_HAND,RIGHT_HAND]

hook_frames_names_meshcat = [LEFT_HAND+"HookFrame", RIGHT_HAND+"HookFrame" ]
hook_frames_names = ["LARM_HOOK", "RARM_HOOK"]


def addframes(viz):
    for framename in FRAME_NAMES:
        viz.addFrame(framename, 0.2)
    for framename in hook_frames_names_meshcat:
        viz.addFrame(framename, 0.2)

def updaterobotframes(viz, robot):
    for jointname, framename in zip(FRAME_JOINT_NAMES,FRAME_NAMES):
        frameid = robot.model.getFrameId(jointname)
        oMframe = robot.data.oMf[frameid] 
        viz.applyConfiguration(framename,oMframe)
        
def updatecubeframes(viz, cube):
    for hookframename, framenamemeshcat in zip(hook_frames_names,hook_frames_names_meshcat):
        viz.applyConfiguration(framenamemeshcat,getcubeplacement(cube,hookframename))

def setupmeshcat(robot, url=MESHCAT_URL):
    if USE_MESHCAT:
        viz = MeshcatVisualizer(robot, url=url)    
        #add target
        viz.addBox("target", [0.1,0.1,0.1],[0.,0.9,0.,0.5])
        viz.applyConfiguration('target', CUBE_PLACEMENT_TARGET)
        viz.loadViewerModel()     
        viz.display(robot.q0)
        viz.displayCollisions(False)
        viz.displayVisuals(True)
        addframes(viz)
        return viz
    else:
        print("MESHCAT viewer is not selected, this method should not be called")
        sys.exit(0)
        

def updatevisuals(viz, robot, cube, q):
    updaterobotframes(viz, robot)
    updatecubeframes(viz,cube)
    viz.display(q)
