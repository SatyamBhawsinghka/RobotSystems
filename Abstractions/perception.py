#!/usr/bin/python3
# coding=utf8
import logging

from Lib.ArmPi.LABConfig import color_range

logging_format = "%(asctime)s: %(message)s"
import atexit
from logdecorator import log_on_start, log_on_end, log_on_error
import cv2
import time
import threading
import math
import numpy as np
import sys
sys.path.append('/../Lib/ArmPi/')
import Camera
from LABConfig import color_range
from ArmIK.Transform import getMaskROI, getROI, getCenter, convertCoordinate
from CameraCalibration.CalibrationConfig import square_length


class Perception(object):
    @log_on_start(logging.DEBUG, "Constructor called ")
    @log_on_error(logging.DEBUG, "Error in constructor call")
    @log_on_end(logging.DEBUG, "Constructor finished")
    def __init__(self, logging_level='INFO'):
        self.camera = Camera.Camera()
        self.target_color = ['red']
        self.isRunning = False
        self.size = (640, 480)
        self.get_roi = False
        self.roi = ()
        self.start_pick_up = False
        self.center_list = []
        self.last_x, self.last_y = 0, 0
        self.world_X, self.world_Y = 0, 0
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.rect = None
        self.count = 0
        self.unreachable = False
        self.rotation_angle = 0
        self.start_count_t1 = True
        self.t1 = 0
        self.detect_color = 'None'
        self.draw_color = self.range_rgb["black"]
        self.color_list = []
        if logging_level == 'INFO':
            logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
        elif logging_level == 'DEBUG':
            logging.basicConfig(format=logging_format, level=logging.DEBUG, datefmt="%H:%M:%S")

        self.camera.camera_open()
        atexit.register(self.stop)

    def sense(self):
        img = self.camera.frame()
        self.isRunning = True
        return img.copy()

    def stop(self):
        self.isRunning = False
        self.camera.camera_close()
        cv2.destroyAllWindows()

    def show(self, frame):
        cv2.imshow('Frame', frame)

    def process(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        # If an area is detected with a recognized object, the area is detected until there are none
        if self.get_roi and not self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0

        if not self.start_pick_up:
            for i in color_range:
                if i in self.target_color:
                    frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # Bitwise operations on the original image and mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[
                        -2]  # find the outline
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour
                    if areaMaxContour is not None:
                        if area_max > max_area:  # find the largest area
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
            if max_area > 2500:  # have found the largest area
                self.rect = cv2.minAreaRect(areaMaxContour_max)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = getROI(box)  # get roi region
                self.get_roi = True
                img_centerx, img_centery = getCenter(self.rect, self.roi, self.size,
                                                     square_length)  # Get the coordinates of the center of the block

                world_x, world_y = convertCoordinate(img_centerx, img_centery,
                                                     self.size)  # Convert to real world coordinates

                cv2.drawContours(img, [box], -1, self.range_rgb[color_area_max], 2)
                cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color_area_max], 1)  # draw center point

                distance = math.sqrt(pow(world_x - self.last_x, 2) + pow(world_y - self.last_y, 2))  # Compare the last coordinates to determine whether to move
                self.last_x, self.last_y = world_x, world_y
                if not self.start_pick_up:
                    if color_area_max == 'red':  # red max
                        color = 1
                    elif color_area_max == 'green':  # green max
                        color = 2
                    elif color_area_max == 'blue':  # blue max
                        color = 3
                    else:
                        color = 0
                    self.color_list.append(color)
                    # Cumulative judgment
                    if distance < 0.5:
                        self.count += 1
                        self.center_list.extend((world_x, world_y))
                        if self.start_count_t1:
                            start_count_t1 = False
                            self.t1 = time.time()
                        if time.time() - self.t1 > 1:
                            self.rotation_angle = self.rect[2]
                            self.start_count_t1 = True
                            self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                            self.center_list = []
                            self.count = 0
                            self.start_pick_up = True
                    else:
                        self.t1 = time.time()
                        self.start_count_t1 = True
                        self.center_list = []
                        self.count = 0

                    if len(self.color_list) == 3:  # multiple judgments
                        # take the average
                        color = int(round(np.mean(np.array(self.color_list))))
                        self.color_list = []
                        if color == 1:
                            self.detect_color = 'red'
                            self.draw_color = self.range_rgb["red"]
                        elif color == 2:
                            self.detect_color = 'green'
                            self.draw_color = self.range_rgb["green"]
                        elif color == 3:
                            self.detect_color = 'blue'
                            self.draw_color = self.range_rgb["blue"]
                        else:
                            self.detect_color = 'None'
                            self.draw_color = self.range_rgb["black"]
            else:
                if not self.start_pick_up:
                    self.draw_color = (0, 0, 0)
                    self.detect_color = "None"

        cv2.putText(img, "Color: " + self.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)
        return img

    def setTargetColor(self, target_color):
        self.target_color = target_color


    # Find the contour with the largest area
    # argument is a list of contours to compare
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # The contour with the largest area is valid only if the area is greater than 300 to filter out the noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # returns the largest contour


if __name__ == '__main__':
    print("Perception starting in a second")
    time.sleep(1)
    percept = Perception()
    while True:
        img = percept.sense
        if img is not None:
            frame = img.copy()
            Frame = percept.process(frame)
            percept.show(Frame)
            key = cv2.waitKey(1)
            if key == 27:
                percept.stop
                print("Perception code ended")
                break





