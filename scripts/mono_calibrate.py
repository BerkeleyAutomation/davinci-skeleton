"""
This script handles the wrist calibration of the robot to image frame.

Specifically, this helps us translate from image coordinates from the dvrk
camera (i.e. pixels) to the robot coordinates. This assumes that the surface of
interest for where the robot will be touching is flat and has a set of clearly
defined circular contours. After the robot and camera are both set up, the data
collector will generate left camera images and right camera images, along with
respective contours. 

Then, during `calibrateImage`, the code sequentially iterates through contours
and displays the selected contour on screen. At that point, the code waits
(using `cv2.waitKey`), and a human must adjust the appropriate arm of the dvrk
(left or right) so that the end-effector lies in the center of the chosen
contour. This process repeats for all subsequent contours, and the data points
are stored in a pickle file. Then, use `mono_calibrate_generate_maps.py` to
figure out a mapping from image pixels to robot coordinates.
"""

import environ

from dvrk.robot import *
from autolab.data_collector import DataCollector

import cv2
import numpy as np
import pickle

def initializeRobots():

    d = DataCollector()

    r1 = robot("PSM1")
    r2 = robot("PSM2")

    time.sleep(2)
    
    return (r1,r2,d)


def storeData(filename, arm1, arm2):
    f = open(filename, 'a')
    pickle.dump((arm1, arm2), f)
    f.close()


def calibrateImage(contours, img, arm1, arm2, filename):

    #wait for everything to load
    arm1.home()
    arm2.home()

    for cX, cY, approx, peri in contours:  

        limg = img.copy()
        rimg = img.copy()

        cv2.circle(limg, (cX, cY), 50, (0,0,255))
        cv2.drawContours(limg , [approx], -1, (0, 255, 0), 3)
        cv2.imshow("Calibration PSM", limg)
        cv2.waitKey(0)

        frame = arm1.get_current_cartesian_position()
        pos = tuple(frame.position[:3])
        rot = tfx.tb_angles(frame.rotation)
        rot = (rot.yaw_deg, rot.pitch_deg, rot.roll_deg)

        arm1.home()


        a1 = (pos, rot, cX, cY)

        cv2.circle(rimg, (cX, cY), 50, (0,0,255))
        cv2.drawContours(rimg , [approx], -1, (0, 255, 0), 3)
        cv2.imshow("Calibration PSM", rimg)
        cv2.waitKey(0)

        frame = arm2.get_current_cartesian_position()
        pos = tuple(frame.position[:3])
        rot = tfx.tb_angles(frame.rotation)
        rot = (rot.yaw_deg, rot.pitch_deg, rot.roll_deg)

        a2 = (pos, rot, cX, cY)

        arm2.home()

        storeData(filename, a1, a2)



if __name__ == "__main__":
    arm1, arm2, d = initializeRobots()

    #left calibration
    calibrateImage(d.left_contours, d.left_image, arm1, arm2, 'config/left.p')

    #right calibration
    #calibrateImage(d.right_contours, d.right_image, arm1, arm2, 'config/right.p')












