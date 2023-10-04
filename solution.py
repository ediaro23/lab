#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  4 11:43:26 2023

@author: stonneau
"""

from os.path import dirname, join, abspath
import numpy as np
import pinocchio as pin #the pinocchio library
from pinocchio.utils import rotate

#helpers 
#if needed, you can store the placement of the right hand in the left hand frame here
LMRREF = pin.SE3(pin.Quaternion(-0,0, 0, 1 ), np.array(np.array([0, 0, 0])))
RMLREF = LMRREF.inverse()
