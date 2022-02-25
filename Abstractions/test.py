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

Board.setBusServoPulse(1, 500, 300)
Board.setBusServoPulse(2, 500, 500)
AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
time.sleep(1.5)
print('reset done')
time.sleep(1)

AK.setPitchRangeMoving((0, 0, 10), -30, -30, -90, 1500)
time.sleep(1.5)
print('moved to 0,0')
time.sleep(1)

AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
time.sleep(1.5)
print('moved to 0,10')
time.sleep(1)

AK.setPitchRangeMoving((10, 10, 10), -30, -30, -90, 1500)
time.sleep(1.5)
print('moved to 10,10')
time.sleep(1)

AK.setPitchRangeMoving((10, 0, 10), -30, -30, -90, 1500)
time.sleep(1.5)
print('moved to 10,0')
time.sleep(1)



