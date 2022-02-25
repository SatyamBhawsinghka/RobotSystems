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

AK = ArmIK()

Board.setBusServoPulse(1, 500, 300)
Board.setBusServoPulse(2, 500, 500)
AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
time.sleep(1.5)
print('reset done')
time.sleep(1)

result = AK.setPitchRangeMoving((0, 10, 10), -90, -90, 0)
if result == False:
    print('unreachable')
else:
    time.sleep(result[2]/1000)
    print('moved to 0,10')
time.sleep(1)

result = AK.setPitchRangeMoving((0, 20, 10), -90, -90, 0)
if result == False:
    print('unreachable')
else:
    time.sleep(result[2]/1000)
    print('moved to 0,20')
time.sleep(1)

result = AK.setPitchRangeMoving((10, 20, 10), -90, -90, 0)
if result == False:
    print('unreachable')
else:
    time.sleep(result[2]/1000)
    print('moved to 10,20')
time.sleep(1)

result = AK.setPitchRangeMoving((-10, 20, 10), -90, -90, 0)
if result == False:
    print('unreachable')
else:
    time.sleep(result[2]/1000)
    print('moved to -10,20')
time.sleep(1)



