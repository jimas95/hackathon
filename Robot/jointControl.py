
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
import time
# The robot object is what you use to control the robot
robot = InterbotixRobot(robot_name="px100", mrd=mrd)



title_window = "OpenCV Stream"
cv2.namedWindow(title_window, cv2.WINDOW_AUTOSIZE)

alpha_slider_max = 100
HueValU = 255
SatValU = 255
ValValU = 255

waist_value = 0
elbow_value = 0
shoulder_value = 0

def on_trackbarHD(val):
    global HueValD
    alpha = val / alpha_slider_max
    beta = ( 1.0 - alpha )
    HueValD = int(alpha*255)

def on_trackbarSD(val):
    global SatValD
    alpha = val / alpha_slider_max
    beta = ( 1.0 - alpha )
    SatValD = int(alpha*255)

def on_trackbarVD(val):
    global ValValD
    alpha = val / alpha_slider_max
    beta = ( 1.0 - alpha )
    ValValD = int(alpha*255)



def on_trackbar_waist(val):
    global waist_value
    alpha = (val-100.0) / alpha_slider_max
    waist_value = alpha
    print("waist_value :" + str(waist_value))

def on_trackbar_elbow(val):
    global elbow_value
    alpha = (val-100.0) / alpha_slider_max
    elbow_value = alpha
    print("elbow_value :" + str(elbow_value))

def on_trackbar_shoulder(val):
    global shoulder_value
    alpha = (val-100.0) / alpha_slider_max
    shoulder_value = alpha
    print("shoulder_value :" + str(shoulder_value))

trackbar_waist = 'waist x %d' % 200
trackbar_elbow = 'elbow x %d' % 200
trackbar_shoulder = 'shoulder x %d' % 200
cv2.createTrackbar(trackbar_waist, title_window , 100, 200, on_trackbar_waist)
cv2.createTrackbar(trackbar_elbow, title_window , 10, 200, on_trackbar_elbow)
cv2.createTrackbar(trackbar_shoulder, title_window , 100, 200, on_trackbar_shoulder)



trackbar_nameHD = 'HueD x %d' % alpha_slider_max
trackbar_nameSD = 'SatD x %d' % alpha_slider_max
trackbar_nameVD = 'ValD x %d' % alpha_slider_max
cv2.createTrackbar(trackbar_nameHD, title_window , 0, alpha_slider_max, on_trackbarHD)
cv2.createTrackbar(trackbar_nameSD, title_window , 0, alpha_slider_max, on_trackbarSD)
cv2.createTrackbar(trackbar_nameVD, title_window , 0, alpha_slider_max, on_trackbarVD)

blank_image = np.zeros((100,1000,3), np.uint8)


# Streaming loop
try:
    while True:

        cv2.imshow(title_window,blank_image)
        key = cv2.waitKey(1)
        robot.set_single_joint_position("waist",   waist_value, blocking=False)
        robot.set_single_joint_position("shoulder",shoulder_value, blocking=False)
        robot.set_single_joint_position("elbow",   elbow_value, blocking=False)
    
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
        robot.go_to_sleep_pose()
        print("End")