#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import Float32


# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

# import blob_param_siljun.py

# end of the blob_param_siljun.py

#import turtle_video_siljun.py
def line_trace(frame,stage,verbose): ### find line then return angular velocity

        cv2.imshow('frame',frame)
        lower_white=np.array([0,0,200]) ### HSV range used in white detect
        upper_white=np.array([180,15,255])
        lower_yellow=np.array([27,75,163])
        upper_yellow=np.array([35,163,225])


    # cvtColor(src, code)
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #BGR -> HSV
        mask_white=cv2.inRange(hsv,lower_white,upper_white)
        white=cv2.bitwise_and(frame,frame,mask=mask_white)

    # yellow line
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask_yellow=cv2.inRange(hsv,lower_yellow,upper_yellow)
        yellow=cv2.bitwise_and(frame,frame,mask=mask_yellow)


        cv2.line(frame,(165,175),(475,175),(253,244,8),2)
        cv2.line(frame,(165,235),(475,235),(253,244,8),2) ### draw ROI
        cv2.line(frame,(165,175),(165,235),(253,244,8),2)
        cv2.line(frame,(475,175),(475,235),(253,244,8),2)

    ### Process image to make finding line easy
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # BGR -> Grayscale
        ROI=gray[180:230,170:470]
        ROI=cv2.GaussianBlur(ROI,(21,21),0)
        thr=cv2.adaptiveThreshold(ROI,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        blur=cv2.medianBlur(thr,9)
        edge=cv2.Canny(blur,180,360)

        left_edge=edge[:,:edge.shape[1]/2] ### in left side, it finds only '/'type when jucha stage
        right_edge=edge[:,edge.shape[1]/2:] ### in right side, it finds only '\'type when jucha stage
        L_lines=cv2.HoughLines(left_edge,1,np.pi/180,30)
        R_lines=cv2.HoughLines(right_edge,1,np.pi/180,30)

        lineL=[] ### value initializing
        lineR=[]
        L=0
        R=0
        i=0
        Ldegree=0
        Rdegree=0

        if R_lines is not None:
                R_lines=[l[0] for l in R_lines]
                for rho,theta in R_lines:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
                        degree=np.arctan2(y2-y1,x2-x1)*180/np.pi
                        if degree>3 and R==0:
                                i+=1
                                Rdegree=degree
                                R+=2
                                cv2.line(frame,(x1+320,y1+180),(x2+320,y2+180),(0,100,100),3)
                                break
                        else:
                                continue

        if L_lines is not None:
                L_lines=[l[0] for l in L_lines]
                for rho,theta in L_lines:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
                        degree=np.arctan2(y2-y1,x2-x1)*180/np.pi
                        if degree<-3 and L==0:
                                i+=1
                                Ldegree=degree
                                L+=2
                                cv2.line(frame,(x1+170,y1+180),(x2+170,y2+180),(0,100,100),3)
                                break
                        else:
                                continue


        if verbose is True: ### discribe the existence of line and angle and number
                print('lineL is')
                print(lineL)
                print(Ldegree)
                print('lineR is')
                print(lineR)
                print(Rdegree)
                print('there is %d lines'%(i))

        if i==2:
                return frame,-(Ldegree+Rdegree)*0.065 ### if there are two lines, then angular_vel depends on difference of angle


        elif i==1:
                if Ldegree==0:
                        return frame,-(Rdegree-90)*0.06

                else:
                        return frame,-(Ldegree+90)*0.06

        else:
                return frame,-0.001

# end of the turtle_video_siljun.py

pub=rospy.Publisher('/cmd_vel',Twist,queue_size=5)
pub_stage=rospy.Publisher('/stage',Int8,queue_size=5)

######################################################################################################
###############################<<< Trainnig parking sign >>>##########################################


###################################################################################
####################<<< Initial Value Definition >>>###############################

time = rospy.Duration(1.6)

angular=0 # initial angular vel


######################################################################################
#################################<<< Funtions >>>#####################################

def turtlestop(): ### Function that stop the turtlebot3 when node is shut down

	twist = Twist()
        twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0

        twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0

        pub.publish(twist)



def turtlemove(linear,angular): ### Function that move the turtlebot

	rospy.on_shutdown(turtlestop)

	twist=Twist()
	twist.linear.x=linear
	twist.angular.z=angular

	pub.publish(twist)


def angular_Selecting(ros_data): #Function setting angular velocity

	global angular; 
	np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	
	
	K = np.array([[  220,     0.  ,  320],
              		[    0.  ,   200,   240],
              		[    0.  ,     0.  ,     1.  ]])

	# zero distortion coefficients work well for this image
	D = np.array([0., 0., 0., 0.])

	# use Knew to scale the output
	Knew = K.copy()
	Knew[(0,1), (0,1)] = 0.5 * Knew[(0,1), (0,1)]
	#cv2.imshow('video_fisheye',image_np)
	img_undistorted = cv2.fisheye.undistortImage(image_np, K, D=D, Knew=Knew)
	
	
	

	####<<< Selecting angular velocity >>>####
	image_np,angular=turtle_video_siljun.line_trace(img_undistorted,stage,verbose=False)
	
	if angular>0.28 or angular<-0.28:
		turtlemove(0.13,angular)
		#print('curve')
	else:
			turtlemove(0.17,angular)
	
		#print('line')	
	#print(angular)
	#########################<<< Show processed image >>>##############################
	cv2.imshow('undistorted', img_undistorted)
	
	cv2.waitKey(1)&0xFF


######################################################################################################

rospy.init_node('video_processor', anonymous=True)
rospy.Subscriber('/camera/image/compressed',CompressedImage, angular_Selecting,  queue_size = 1)

rospy.spin()
