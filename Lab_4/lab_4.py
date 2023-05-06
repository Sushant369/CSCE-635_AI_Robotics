#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped
            # p.publish(NatuStep(p))
import concurrent.futures
# Python 2/3 compatibility imports
import rospy

from threading import *
from time import *
from math import pi, fabs, cos, sin

###############################################################################
#robot controller / state manager
###############################################################################

def twistFromDirection(self,joint_goals):
    twist = TwistStamped()
    joint_limits = [20, 20, 20, 20] #angle limits per-joint
    joint_goal = [x * lim for x, lim in zip(joint_goals, joint_limits)]

    # print(joint_goal[0] ,joint_goal[1],joint_goal[2],joint_goal[3])
    twist.twist.linear.x = joint_goal[0] #base forward/back
    twist.twist.linear.y = -joint_goal[1] #base yaw (left/right)
    twist.twist.linear.z = joint_goal[2] #head roll
    twist.twist.angular.x = -joint_goal[3] #head pitch (up/down)

    return twist


def Step2_1_2bot(self):
    check=True
    twist = TwistStamped()
    twist.twist.linear.x = 0#base forward/back
    twist.twist.linear.y = -0 #base yaw (left/right)
    twist.twist.linear.z = 0 #head roll
    twist.twist.angular.x = -0 #head pitch (up/down)
    self.publish(twist)
    sleep(1)
    nt=rospy.get_time()


    while True:
            
        twist.twist.linear.x = 0#base forward/back
        twist.twist.linear.z = 0 #head roll
        twist.twist.angular.x = -0 #head pitch (up/down)
        for i in range(0,20,1):
            twist.twist.linear.y = -i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  

        for i in range(0,20,1):
            twist.twist.linear.y = i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  


        nt_new=rospy.get_time()
        if nt_new-nt >= 18:
            break  


def Step2_3_4bot(self):
    check=True
    twist = TwistStamped()
    twist.twist.linear.x = 0#base forward/back
    twist.twist.linear.y = -0 #base yaw (left/right)
    twist.twist.linear.z = 0 #head roll
    twist.twist.angular.x = -0 #head pitch (up/down)
    self.publish(twist)
    sleep(1)
    nt=rospy.get_time()


    while True:
            
        twist.twist.linear.x = 0#base forward/back
        twist.twist.linear.z = 0 #head roll
        twist.twist.angular.x = -0 #head pitch (up/down)
        for i in range(0,20,1):
            twist.twist.linear.y = i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  

        for i in range(0,20,1):
            twist.twist.linear.y = -i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  


        nt_new=rospy.get_time()
        if nt_new-nt >= 18:
            break  



def NatuStep(self):
    check=True
    twist = TwistStamped()
    twist.twist.linear.x = 0#base forward/back
    twist.twist.linear.y = -0 #base yaw (left/right)
    twist.twist.linear.z = 0 #head roll
    twist.twist.angular.x = -0 #head pitch (up/down)
    self.publish(twist)
    sleep(1)
    nt=rospy.get_time()


    while True:
            
        twist.twist.linear.x = 0#base forward/back
        twist.twist.linear.y = -0 #base yaw (left/right)
        twist.twist.linear.z = 0  #head roll

        for i in range(0,15,1):
            
            twist.twist.angular.x = -i #head pitch (up/down)
            self.publish(twist)
            r.sleep()
   
        nt_new=rospy.get_time()
        if nt_new-nt >= 2.5:
            break  
    while True:
            
        twist.twist.linear.x = 0#base forward/back
        twist.twist.linear.z = 0 #head roll
        twist.twist.angular.x = -0 #head pitch (up/down)
        for i in range(0,20,1):
            twist.twist.linear.y = -i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  

        for i in range(0,20,1):
            twist.twist.linear.y = i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  

        nt_new=rospy.get_time()
        if nt_new-nt >= 2.5:
            break  




def botsync(self):
    twist = TwistStamped()
    nt=rospy.get_time()
    while True:
            
        twist.twist.linear.x = 0#base forward/back
        twist.twist.linear.z = 0 #head roll
        twist.twist.angular.x = -0 #head pitch (up/down)
        for i in range(0,20,1):
            twist.twist.linear.y = -i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  

        for i in range(0,20,1):
            twist.twist.linear.y = i #base yaw (left/right)
            self.publish(twist)
            r.sleep()  

        nt_new=rospy.get_time()
        if nt_new-nt >= 2.5:
            break  




class RoboController(object):
    global check
    """
    robot controller class; contains behaviors and combines them
    """
    def __init__(self):
        rospy.loginfo("Initializing publishers")

        self.n = 4
        self.pub = [rospy.Publisher(('/sb_' + str(i) + '_cmd_state'), TwistStamped, queue_size=10) for i in range(self.n)]


        rospy.loginfo("Done!")



    #called by __main__ spin loop
    def callback_update(self, time, dt):
        
        periods = [1, 2, 3, 4]
        speed = 0.4
        sin_of = lambda period, phase: sin((time * speed + phase / period) * pi * period)
        #
        for i in range(self.n):
            p = self.pub[i]
            phases = [i] * 4
            js = [sin_of(period, phase) for period, phase in zip(periods, phases) ]
            # print(js)
            p.publish(twistFromDirection(p,js))
    
    def executestep2(self):
        
        p1 = self.pub[0]
        p2=self.pub[1]
        p3=self.pub[2]
        p4=self.pub[3]

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            executor.submit(Step2_1_2bot, p1)
            executor.submit(Step2_1_2bot, p2)
            executor.submit(Step2_3_4bot, p3)
            executor.submit(Step2_3_4bot, p4)        
    

    def callbackNatu(self):
        pool = concurrent.futures.ThreadPoolExecutor(max_workers=4)
        for i in range(self.n):
            p = self.pub[i]
            pool.submit(NatuStep(p))
    


    def sync(self):
        p1 = self.pub[0]
        p2=self.pub[1]
        p3=self.pub[2]
        p4=self.pub[3]

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            executor.submit(botsync, p1)
            executor.submit(botsync, p2)
            executor.submit(botsync, p3)
            executor.submit(botsync, p4)




###############################################################################
#main run loop
###############################################################################

if __name__ == '__main__':
    rospy.init_node("lab_4_node")
    rospy.loginfo("Lab 4 node initialized")

    robot = RoboController()

    #run loop
    r = rospy.Rate(60)
    t = rospy.get_time()
    current_time = rospy.get_time()

    while not rospy.is_shutdown():
        dt = rospy.get_time() - t
        t = rospy.get_time()
        robot.callback_update(t, dt)
        print(current_time - t)
        r.sleep()    
        if t - current_time >= 11:
            break 
    
    robot.executestep2()

    # robot.
    robot.callbackNatu()
    robot.sync()
    current_time = rospy.get_time()
    while True:
        dt = rospy.get_time() - t
        t = rospy.get_time()
        robot.callback_update(t, dt)
        print(current_time - t)
        r.sleep()    
        if t - current_time >= 22.1:
            break 
    robot.executestep2()
    robot.callbackNatu()
    robot.sync()    
    current_time = rospy.get_time()
    while True:
        dt = rospy.get_time() - t
        t = rospy.get_time()
        robot.callback_update(t, dt)
        print(current_time - t)
        r.sleep()    
        if t - current_time >= 8:
            break 


