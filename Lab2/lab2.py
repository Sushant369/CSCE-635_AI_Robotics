#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import TwistStamped


# Python 2/3 compatibility imports
import sys
import copy
import math
import rospy
import time
import geometry_msgs.msg
import moveit_commander

import cv2
import mediapipe as mp
import numpy as np
import threading 
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String

twist = TwistStamped()
Counter_1=0
Counter_2=0
timer=-1
face_detected=True
clap_detected=False
current_frame=None

mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.6)
defaultFlag=True
ycheckflag=False
timer1=0
def get_dist(a,b):
    return math.sqrt( (a.x-b.x)**2 +(a.y-b.y)**2   )



class GenericBehavior(object):
    """
    Generic behavior class.
    """
    def __init__(self):
        self.pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
        self.twist_pub = rospy.Publisher('/sb_cmd_state', TwistStamped, queue_size=10)
        print("generic class")
        self.audio_sub = rospy.Subscriber("/audio", Float32MultiArray, callback=self.audio_callback, queue_size=1)
        print("executed audio_sub")
        self.camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=self.video_callback, queue_size=1)
        rospy.loginfo("Node started.")
        self.group_name = "survivor_buddy_head"

#########Bring bot to default position############
    def execute_default(self):     
        print("Default Executed.....")      
        twist.twist.linear.x =  -30.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = -10.0
        self.twist_pub.publish(twist)
        time.sleep(1)

##############Clap detection###########################
    def audio_callback(self, data):
        # print("inside Audio callback")
        global clap_detected,defaultFlag

        if defaultFlag:
            self.execute_default()
            defaultFlag=False
        else:
            if clap_detected==True:
                return
            elif self.detect_noise(data):

                self.audio_behavior(True)
                clap_detected=True

    def audio_behavior(self,a):
    # joint value planning
        global twist
        if( a ):
            # print('inside audio_behavior')
            twist.twist.linear.x = -0.0
            twist.twist.linear.y = -0.0
            twist.twist.linear.z = 0.0
            twist.twist.angular.x = -0.0
            self.twist_pub.publish(twist)
            time.sleep(1)

    def detect_noise(self,data):
        print('detect noise method')
        #print(data.data)
        alert = False
        rms = float(np.sqrt(np.mean(data.data)**10))
        rms= math.log2(rms)
        rms=rms*-1
        print(rms)
        # if np.average(np.absolute(data.data))>0.0015 and max(data.data)>0.8:
        if rms<50.0:
            print("Clap detected")
            alert=True
        else:
            print("Clap not detected")
        return alert

############Face detection and movement#####################
    def video_callback(self, data):
        global clap_detected,ycheckflag
        if clap_detected==False:
            return
        else:
            global timer,timer1 , Counter_1, Counter_2, face_detected
            x=0.0
            y=0.0
            width=0.0
            height=0.0

            global current_frame,VideoData
            VideoData=data

            if timer1 !=0:
                timer1 -=1
                # time.sleep(0.5)
                return
            else:
                timer1=8

                np_arr = np.frombuffer(data.data,np.uint8)
                cv_frame = cv2.imdecode(np_arr, 1)
                image=cv2.resize(cv_frame,(64,64))

                if timer==-1:
                    timer=int(time.time())


                # results = face_detection.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                results = face_detection.process(image)
                if results.detections:
                    for detection in results.detections:

                        # Draw bounding boxes around the detected faces
                        if results.detections:
                            for detection in results.detections:
                                bbox = detection.location_data.relative_bounding_box
                                h, w, _ = image.shape
                                x, y, width, height = int(bbox.xmin * w), int(bbox.ymin * h), \
                                                    int(bbox.width * w), int(bbox.height * h)
                                cv2.rectangle(image, (x, y), (x+width, y+height), (0, 255, 0), 2)

                        timer=int(time.time())
                        face_detected=True
                        current_frame=image
                        print("center_x", x+width//2)
                        print("center_y",y+height//2)
                        if ycheckflag==False:
                            ycheck=  center_y
                        center_x = x+width//2
                        center_y = y+height//2
                        if (center_x <28):
                            print("move right")
                            if (twist.twist.linear.y <= 45.0):
                                twist.twist.linear.y += -5.0

                                self.twist_pub.publish(twist)
                                # rate.sleep() 
                        elif (center_x>33):
                            print("move left")
                            if (twist.twist.linear.y )>= -45.0:
                                twist.twist.linear.y -= -5.0
                                self.twist_pub.publish(twist)
                                # rate.sleep() 

                        elif (center_y>32):
                            if (twist.twist.linear.x )<= 45.0:
                                print("move back")  
                                twist.twist.linear.x -= -5.0


                                self.twist_pub.publish(twist)
                              

                        elif (center_y<27):
                            if (center_y< 20):
                                # print("crower")
                                if (twist.twist.linear.x )<= 45:
                                    print("crower")
                                    twist.twist.linear.x = 15
                                    self.twist_pub.publish(twist)     
                            else:
                                if (twist.twist.linear.x )>= -45:
                                    print("move front")
                                    twist.twist.linear.x += -2.0

                                    self.twist_pub.publish(twist)
     

                elif face_detected and time.time()-int(timer)>=2:
                    print("Search for Sushant")
                    if (twist.twist.linear.y >= -45.0):
                        twist.twist.linear.y = 10.0
                        self.twist_pub.publish(twist)
                        time.sleep(1)
                        twist.twist.linear.y = -0.0 
                        self.twist_pub.publish(twist)   
                        time.sleep(1)                   
                        twist.twist.linear.y = -10.0
                        self.twist_pub.publish(twist)
                    timer=int(time.time())
                    # face_detected=False
                    

            cv2.imshow("Face detect",image)
            cv2.waitKey(3)
            pass



    # Create and start the thread for displaying images

def display_image():
    print('inside display_image')
    global current_frame
    if clap_detected == False:
        print('return display')
        pass
    else:
        # if current_frame is not None:
        while True:
            if current_frame is not None:
                cv2.imshow("Face Detection", current_frame)
                cv2.waitKey(3)
    # else:
    #     return

if __name__ == '__main__':
    rospy.init_node("lab_2_node")
    moveit_commander.roscpp_initialize(sys.argv)
    GenericBehavior()
# while True:
    # Generic_thread=threading.Thread(target=GenericBehavior)
    # display_thread = threading.Thread(target=display_image)
    # # Generic_thread.start()
    # display_thread.start()  

    rospy.spin()
