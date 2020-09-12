## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# config.enable_record_to_file("camera_video")
config.enable_device_from_file("camera_video2")

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

title_window = "OpenCV Stream"
cv2.namedWindow(title_window, cv2.WINDOW_AUTOSIZE)

alpha_slider_max = 100
HueValU = 255 #int(0.13 * 255)
SatValU = 255 #int(0.30 * 255)
ValValU = 255 #int(0.28 * 255)

HueValD = 0
SatValD = 0
ValValD = 0

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



def on_trackbarHU(val):
    global HueValU
    alpha = val / alpha_slider_max
    beta = ( 1.0 - alpha )
    HueValU = int(alpha*255)

def on_trackbarSU(val):
    global SatValU
    alpha = val / alpha_slider_max
    beta = ( 1.0 - alpha )
    SatValU = int(alpha*255)

def on_trackbarVU(val):
    global ValValU
    alpha = val / alpha_slider_max
    beta = ( 1.0 - alpha )
    ValValU = int(alpha*255)

trackbar_nameHU = 'HueU x %d' % alpha_slider_max
trackbar_nameSU = 'SatU x %d' % alpha_slider_max
trackbar_nameVU = 'ValU x %d' % alpha_slider_max
cv2.createTrackbar(trackbar_nameHU, title_window , 100, alpha_slider_max, on_trackbarHU)
cv2.createTrackbar(trackbar_nameSU, title_window , 100, alpha_slider_max, on_trackbarSU)
cv2.createTrackbar(trackbar_nameVU, title_window , 100, alpha_slider_max, on_trackbarVU)



trackbar_nameHD = 'HueD x %d' % alpha_slider_max
trackbar_nameSD = 'SatD x %d' % alpha_slider_max
trackbar_nameVD = 'ValD x %d' % alpha_slider_max
cv2.createTrackbar(trackbar_nameHD, title_window , 0, alpha_slider_max, on_trackbarHD)
cv2.createTrackbar(trackbar_nameSD, title_window , 0, alpha_slider_max, on_trackbarSD)
cv2.createTrackbar(trackbar_nameVD, title_window , 0, alpha_slider_max, on_trackbarVD)

def pen_recognision(img):
    cv2.circle(img,(447,63), 63, (128,10,120), -1)
    cv2.rectangle(img,(384,0),(510,128),(128,0,128),3)
    hsvImage = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_purple = np.array([HueValD, SatValD, ValValD])
    upper_purple = np.array([HueValU, SatValU, ValValU])

    mask = cv2.inRange(hsvImage, lower_purple, upper_purple)
    res = cv2.bitwise_and(img, img, mask=mask)

    return res






# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)



        proc_image = pen_recognision(color_image)


        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, proc_image))

        cv2.imshow(title_window, images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()