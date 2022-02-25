#!/usr/bin/python3
# coding=utf8
import logging
logging_format = "%(asctime)s: %(message)s"
import atexit
from logdecorator import log_on_start, log_on_end, log_on_error
import cv2
import time
import math
import numpy as np
import sys
sys.path.append('../Lib/ArmPi/')
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
sys.path.append('../../Abstractions/')
from perception import Perception
percept = Perception()
AK = ArmIK()

result = AK.setPitchRangeMoving((percept.world_X, percept.world_Y, 7), -90, -90, 0)
if result == False:
    print("Unreachable")
else:
    time.sleep(result[2]/1000)
    print("reached position", percept.world_X, percept.world_Y)
time.sleep(2)

result = AK.setPitchRangeMoving((0, 0, 7), -90, -90, 0)
if result == False:
    print("Unreachable")
else:
    time.sleep(result[2]/1000)
    print("reached position", 0, 0)
time.sleep(2)




