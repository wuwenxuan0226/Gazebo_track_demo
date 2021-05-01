#!/usr/bin/env python

import cv2
import rospy
import sys
import numpy as np
from sensor_msgs.msg import Image
from csi_camera import CSI_Camera
from cv_bridge import CvBridge

# Read a frame from the camera, and draw the FPS on the image if desired
# Return an image
def read_camera(csi_camera):
    #Reads camera into a different CPU thread
    _ , camera_image=csi_camera.read()
    return camera_image

# Initialize camera
# Good for 1280x720
DISPLAY_WIDTH=640
DISPLAY_HEIGHT=360

# 1280x720, 60 fps
SENSOR_MODE_720=4

camera = CSI_Camera()
camera.create_gstreamer_pipeline(
        sensor_id=0,
        sensor_mode=SENSOR_MODE_720,
        framerate=30,
        flip_method=0,
        display_height=DISPLAY_HEIGHT,
        display_width=DISPLAY_WIDTH,
)
camera.open(camera.gstreamer_pipeline)
camera.start()

#Initialize cvBridge
bridge = CvBridge()

fdist = open('/home/cranfield/Group_2_Jetracer/jetracerGazebo_ws/src/jetracer_line_detect/Cam/CalibrationParam.txt', 'r')
d_dist = fdist.read()
d_dist = eval(d_dist)
cam_mtx = d_dist['Camera Matrix']
dist_coeff = d_dist['Disrtortion Coefficients']

fpers = open('/home/cranfield/Group_2_Jetracer/jetracerGazebo_ws/src/jetracer_line_detect/Cam/Perspective_Correction_Param.txt', 'r')
d_pers = fpers.read()
d_pers = eval(d_pers)
bev_mtx = d_pers['Bird Eye Matrix']
frame_dim = d_pers['Bird Eye View Dimensions']
horizon = d_pers['Horizon']

cam_mtx = np.asarray(cam_mtx)
dist_coeff = np.asarray(dist_coeff)
bev_mtx = np.asarray(bev_mtx)

#topic to image
frame = read_camera(camera)
h,  w = frame.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_coeff, (w, h), 1, (w, h))

pub = rospy.Publisher('jetracer/camera1/image_raw', Image, queue_size=10)
rospy.init_node('jetracer_image', anonymous=True)
rate = rospy.Rate(10)

def image_processing():
    while not rospy.is_shutdown():
        frame = read_camera(camera)

        # undistort frame
        dst = cv2.undistort(frame, cam_mtx, dist_coeff, None, newcameramtx)

        # crop the undistorted image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        #Crop undistorted frame from the horizon to the bottom
        dst = dst[horizon:]

        image_message = bridge.cv2_to_imgmsg(dst, encoding="bgr8")

        pub.publish(image_message)
        #show the cv2 image, for testing
        cv2.imshow('frame', dst)
        cv2.waitKey(3)
        rate.sleep()

def main():
    try:
        image_processing()
    except rospy.ROSInterruptException:
        pass
    print('Shutting down')
    camera.stop()
    camera.release()
    cv2.destroyAllWindows()
    sys.exit()

if __name__ == '__main__':
    main()
