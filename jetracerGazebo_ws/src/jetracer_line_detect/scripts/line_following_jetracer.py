#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
from csi_camera import CSI_Camera
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

def throttle_action(inputSignal, maxthrottle, speed_at_max_steering):
    ## Maps [-1,1] signal into a [0,1] range following a parametrized -abs(x) function
    gain = (1-speed_at_max_steering/maxthrottle)
    action = (-abs(gain*inputSignal)+1)*maxthrottle
    return action

def region_of_interest(img, vertices):
    ## Gets an image and makes a masked version of it depending on the vertices provided
    blank = np.zeros_like(img)
    match_mask_color = 255  ## White = 255 ==> mask all black except white poly
    cv2.fillPoly(blank, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, blank)
    return masked_image

def variable_steer_share(carSpeed, maxPerc, minPerc, maxThrottle, minThrottle):
    steeringShare = ((maxPerc-minPerc)/(maxThrottle-minThrottle)) * (carSpeed + maxThrottle) + minPerc
    return steeringShare

def pub_control(control_steering,control_throttle):
    pub_steering = rospy.Publisher('steering_control_action', Float64, queue_size = 10)
    pub_throttle = rospy.Publisher('throttle_control_action', Float64, queue_size = 10)
    pub_throttle.publish(control_throttle)
    pub_steering.publish(control_steering)

def callback(data):
    global throttleAction, steeringAction, P, ierror, steeringPercentage, topError 
    global bottError, pastError_throt, pastTime_throt, pastError, pastTime
    global lineDetectVar

    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    # Bird Eye View
    bev_img = cv2.warpPerspective(frame, bev_mtx, frame_dim)
    #Get bottom point after Bird Eye View projective warp to define BEV image centreline
    dst_bott_half = [[int(frame.shape[1]/2), frame.shape[0]]]
    bev_bott_half = cv2.perspectiveTransform(np.float32([dst_bott_half]), bev_mtx)[0][0]
    bott_half_x = int(bev_bott_half[0])

    #Do a gaussian blur to eliminate noise
    gauss_img = cv2.GaussianBlur(bev_img, (5, 5), 0)

    #Get a mask depending on the line color
    hsv_img = cv2.cvtColor(gauss_img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_img, lower_color, upper_color)

    # Do image opening: Erosion to kill noise, dilate to regaing initial shape without noise
    kernel = np.ones((13, 13), np.uint8)
    mask_ero = cv2.erode(mask, kernel, iterations=1)
    mask_dil = cv2.dilate(mask_ero, kernel, iterations=1)
                
    #Divide mask into top and bottom
    bott_roi_img = mask_dil.copy()
    top_roi_img = mask_dil.copy()
    top_roi_img = region_of_interest(top_roi_img, top_ROI_vertices)
    bott_roi_img = region_of_interest(bott_roi_img, bott_ROI_vertices)
                
    #Find the line contours in each masked images
    contours_top, h = cv2.findContours(top_roi_img, 1, cv2.CHAIN_APPROX_SIMPLE)
    contours_bott, hierarchy = cv2.findContours(bott_roi_img, 1, cv2.CHAIN_APPROX_SIMPLE)
    # Find the biggest contour (if detected)
    if len(contours_bott) > 0 and len(contours_top) > 0:
        #if it can see lines in both halves
        lineDetectVar = 1
        ##### Bottom contour analysis #####
        c = max(contours_bott, key=cv2.contourArea)
        M = cv2.moments(c)
        #Get contour centroid
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = int(M['m10'] / 0.00001)
            cy = int(M['m01'] / 0.00001)

        #Draw useful information in BEV image
        cv2.circle(bev_img, (cx,cy), 5, (255, 0, 0), -1)
        cv2.drawContours(bev_img, contours_bott, -1, (0, 255, 0), thickness=1)
        cv2.line(bev_img, (bott_half_x, 0), (bott_half_x, frame_dim[1]), (0, 0, 255), thickness=1)
        cv2.line(bev_img, (bott_half_x, cy), (cx, cy), (255, 0, 0), thickness=1)

        ### Steering PID implementation
        bottError = cx - bott_half_x #negative in the left plane, positive in the right
        bottError = (2.0/350.0)*bottError #convert error from pixels to [-1,1] range

        ############### Top contour analysis  ###################
        c_top = max(contours_top, key=cv2.contourArea)
        M = cv2.moments(c_top)
        #Get contour centroid
        if M['m00'] != 0:
            cx_top = int(M['m10'] / M['m00'])
            cy_top = int(M['m01'] / M['m00'])
        else:
            cx_top = int(M['m10'] / 0.00001)
            cy_top = int(M['m01'] / 0.00001)

        #Draw useful information in BEV image
        cv2.circle(bev_img, (cx_top,cy_top), 5, (255, 0, 0), -1)
        cv2.drawContours(bev_img, contours_top, -1, (0, 255, 0), thickness=1)
        cv2.line(bev_img, (bott_half_x, 0), (bott_half_x, frame_dim[1]), (0, 0, 255), thickness=1)
        cv2.line(bev_img, (bott_half_x, cy_top), (cx_top, cy_top), (255, 0, 0), thickness=1)

        ### Top Control action implementation
        topError = cx_top - bott_half_x #negative in the left plane, positive in the right
        topError = (2.0/450.0)*topError #convert error from pixels to [-1,1] range
                      
        ######## Command steering and throttle ########
        #Steering depends on the the bottom control action and the top error
        combError = steeringPercentage*bottError+(1-steeringPercentage)*topError
        cv2.putText(bev_img, 'Error ' + str(combError), (10, frame_dim[1]-80), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1)
        ierror += combError
        controlAction = P * combError + I*ierror + D * ((combError-pastError)/(time.time()-pastTime))                
        #update pastTime and pastError
        pastError = combError
        pastTime = time.time()

        if controlAction >=1:
            controlAction = 1
        elif controlAction<=-1:
            controlAction = -1
            
        topControlAction = P_throt*topError + D_throt * ((topError-pastError_throt)/(time.time()-pastTime_throt))#scales up error so that max vel is hardly achieved
        #update pastTime and pastError
        pastError_throt = topError
        pastTime_throt = time.time()
                    
        if topControlAction >=1:
                topControlAction = 1
        elif topControlAction<=-1:
                topControlAction = -1

        steeringAction = controlAction
        #Throttle only depends on the top control action
        throttleAction = throttle_action(topControlAction, maxThrot, minThrot)
                    
    elif len(contours_bott) > 0 and not len(contours_top) > 0:
        ## if it can see thr bottom line but not the top one
        lineDetectVar = 0
        print("I can't see the top line")
        c = max(contours_bott, key=cv2.contourArea)
        M = cv2.moments(c)

        #Get contour centroid
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = int(M['m10'] / 0.00001)
            cy = int(M['m01'] / 0.00001)

        #Draw useful information in BEV image
        cv2.circle(bev_img, (cx,cy), 5, (255, 0, 0), -1)
        cv2.drawContours(bev_img, contours_bott, -1, (0, 255, 0), thickness=1)
        cv2.line(bev_img, (bott_half_x, 0), (bott_half_x, frame_dim[1]), (0, 0, 255), thickness=1)
        cv2.line(bev_img, (bott_half_x, cy), (cx, cy), (255, 0, 0), thickness=1)

        ### Steering PID implementation
        bottError = cx - bott_half_x #negative in the left plane, positive in the right
        bottError = (2.0/350.0)*bottError #convert error from pixels to [-1,1] range
        ierror += bottError

        #PI control action
        controlAction = P*bottError + I*ierror + D * ((bottError-pastError)/(time.time()-pastTime))                
        #update pastTime and pastError
        pastError = bottError
        pastTime = time.time()

        #Saturate control action
        if controlAction >=1:
                controlAction = 1
        elif controlAction<=-1:
                controlAction = -1                
        ### Steer car with control action
        steeringAction = controlAction
        ### Throttle at minimum since top line is missing (possible curve)
        throttleAction = minThrot
                    
    elif len(contours_top) > 0 and not len(contours_bott) > 0:
        ## If it can see the top line but not the bottom line
        lineDetectVar = -1
        print("I can't see the bottom line")
        c_top = max(contours_top, key=cv2.contourArea)
        M = cv2.moments(c_top)
        #Get contour center
        if M['m00'] != 0:
            cx_top = int(M['m10'] / M['m00'])
            cy_top = int(M['m01'] / M['m00'])
        else:
            cx_top = int(M['m10'] / 0.00001)
            cy_top = int(M['m01'] / 0.00001)

        #Draw useful information in BEV image
        cv2.circle(bev_img, (cx_top,cy_top), 5, (255, 0, 0), -1)
        cv2.drawContours(bev_img, contours_top, -1, (0, 255, 0), thickness=1)
        cv2.line(bev_img, (bott_half_x, 0), (bott_half_x, frame_dim[1]), (0, 0, 255), thickness=1)
        cv2.line(bev_img, (bott_half_x, cy_top), (cx_top, cy_top), (255, 0, 0), thickness=1)

        ### Top Control action implementation
        topError = cx_top - bott_half_x #negative in the left plane, positive in the right
        topError = (2.0/450.0)*topError #convert error from pixels to [-1,1] range
        ierror += topError
        topSteerAction = P*topError + I * ierror + D * ((topError-pastError)/(time.time()-pastTime))                
        #update pastTime and pastError
        pastError = topError
        pastTime = time.time()
                    
        topControlAction = P_throt*topError + D_throt * ((topError-pastError_throt)/(time.time()-pastTime_throt))#scales up error so that max vel is hardly achieved
        #update pastTime and pastError
        pastError_throt = topError
        pastTime_throt = time.time()
                    
        #Saturate top Control Action and top Steer Action
        if topSteerAction >=1:
                topSteerAction = 1
        elif topSteerAction<=-1:
                topSteerAction = -1
                    
        if topControlAction >=1:
                topControlAction = 1
        elif topControlAction<=-1:
                topControlAction = -1

        #Car steering control fully by the top error as bottom line is not detected
        steeringAction = topSteerAction
        ### Set car throttle 
        throttleAction = throttle_action(topControlAction, maxThrot, minThrot)
                
    else:
        lineDetectVar = -2
        #If no line is detected in neither of the halves
        print("I don't see ANY line")
        #Set car steering at -1 or 1 depending on the control action sign
        steeringAction = float(np.sign(steeringAction))
        #Set car throttle to the minimum possible
        throttleAction = minThrot

    cv2.putText(bev_img, 'Steering Command ' + str(steeringAction), (10, frame_dim[1]-60), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1)
    cv2.putText(bev_img, 'Throttle Command ' + str(throttleAction), (10, frame_dim[1]-40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1)

    pub_control(steeringAction,throttleAction)
    ### Adapt P coefficient with velocity
    P = ((maxP - minP)/(maxThrot - minThrot))*(throttleAction + maxThrot) + minP
    ### Adapt steering Share with velocity
    steeringPercentage = variable_steer_share(throttleAction, maxPercentage, minPercentage, maxThrot, minThrot)
        
    ##Log the physics data
    log_dict['Speed'].append(throttleAction)
    log_dict['Time'].append(time.time()-iniTime)
    log_dict['Steering'].append(steeringAction)

    ##Log the code data
    log_code_dict['TopError'].append(topError)
    log_code_dict['BottError'].append(bottError)
    log_code_dict['SteeringPercentage'].append(steeringPercentage)
    log_code_dict['P_coeff'].append(P)
    log_code_dict['LineDetectionState'].append(lineDetectVar)

    cv2.imshow("Image window", bev_img)
    cv2.waitKey(3)

fpers = open('/home/cranfield/Group_2_Jetracer/jetracerGazebo_ws/src/jetracer_line_detect/Cam/Perspective_Correction_Param.txt', 'r')
d_pers = fpers.read()
d_pers = eval(d_pers)
bev_mtx = d_pers['Bird Eye Matrix']
bev_mtx = np.asarray(bev_mtx)
frame_dim = d_pers['Bird Eye View Dimensions']
horizon = d_pers['Horizon']

f = open('/home/cranfield/Group_2_Jetracer/jetracerGazebo_ws/src/jetracer_line_detect/Cam/Line_Color.txt', 'r')
data = eval(f.read())
lower_color, upper_color = data['Color Range']
lower_color = np.array(lower_color)
upper_color = np.array(upper_color)   

#Define steering Action and Throttle Action commands
steeringAction = 0
throttleAction = 0

#PID coefficients for steering
P = 0.3
##Define P coefficient at max and min Throttle
maxP = 1
minP = 0.2
I = 0
D = 0.05
ierror = 0
pastError = 0
pastTime = 0

#Throttle parameters
maxThrot = 1
minThrot = 0.35
#Throttle PID
P_throt = 1
D_throt = 0.6
pastError_throt = 0
pastTime_throt = 0

#Combined action (Trust more one point than the other)
steeringPercentage = 0.8 # 1 = Full steering commanded by bottom point
#0 = Full steering commanded by top point
##Define Percentage at max and min Throttle
maxPercentage = 0.5
minPercentage = 0

# Top Region of interest vertices
top_ROI_vertices = np.array([[
    (0, 0),
    (frame_dim[0], 0),
    (frame_dim[0], int(frame_dim[1]/2)),
    (0, int(frame_dim[1]/2)),
]], np.int32)
# Bottom Region of interest vertices
bott_ROI_vertices = np.array([[
    (0, int(frame_dim[1]/2)),
    (frame_dim[0], int(frame_dim[1]/2)),
    (frame_dim[0], frame_dim[1]),
    (0, frame_dim[1]),
]], np.int32)

#variable to know what detection is being done
lineDetectVar = 1 #1 both, 0 bott, -1 top, -2 None
#Log speed and time, and calculate acceleration from those
iniTime = time.time()
log_dict = {'Time': [time.time()-iniTime], 'Speed': [0], 'Steering': [0]}
#Log useful info from code
log_code_dict = {'TopError':[0], 'BottError':[0], 'SteeringPercentage':[steeringPercentage], 'P_coeff': [P], 'LineDetectionState': [lineDetectVar]}
topError = 1
bottError = 1

bridge = CvBridge()
rospy.init_node('image_gazebo_control', anonymous=True)
rate = rospy.Rate(10)
rospy.Subscriber("jetracer/camera1/image_raw", Image, callback)

def main():
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
      pass
  print('Shutting down')
  cv2.destroyAllWindows()
  sys.exit()

if __name__ == '__main__':
    main()