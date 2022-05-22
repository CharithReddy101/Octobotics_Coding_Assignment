#!/usr/bin/env python3

import rospy
import rospkg
import pygame as pg
import sys
from math import sin, cos, pi
from inverted_pendulum_sim.msg import CurrentState
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest

'''
pygame initialization and parameter settings
'''

class Pendulum:
    """
    This class simulates the inverted pendulum.
    """

    def __init__(self):
        """
        @brief: Constructor - Initializes the inverted pendulum simulation.
        """
        self.x = 0
        self.x_dot = 0
        self.x_d_dot = 0
        self.theta = 3.14
        self.theta_dot = 0
        self.theta_d_dot = 0
        self.length = 300
        self.cart_m = 0.5
        self.pendulum_m = 2
        self.g = -9.8
        self.start = rospy.get_time()
        self.force = 0
        self.cur_theta =0.0
        self.current_state_sub = None
        self.ctrl_force_pub = None
        now = rospy.get_rostime()
        self.init_pubs_subs()
        base_set = rospy.ServiceProxy( '/inverted_pendulum/set_params', SetParams )
        base_set(2.0, 300, 0.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0)


        '''
        starting main loop
        '''
        self.main_loop()

    def init_pubs_subs(self):
    
        self.ctrl_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce , queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState,
                                               self.current_state_callback)


    def current_state_callback(self, msg):
   
        self.cur_theta = msg.curr_theta
        rospy.loginfo(self.cur_theta)

        self.cur_x = msg.curr_x


    def main_loop(self):
        """
        @brief: Main loop of the simulation.
        :return: none
        """
        last_time = 0.0
        prev_error = 0.14
        sample_time = 1
        total_error =0
        delta_error=0
        while not rospy.is_shutdown():
             #print(self.theta)
             #print(self.theta_dot)
             theta_error =0.0
             curr_time = rospy.get_time()*1000
             del_time = curr_time - last_time
            
             # if self.cur_theta < 0:
             #     theta_error = -self.cur_theta- self.theta
             # else:
             theta_error = self.theta - self.cur_theta
             if abs(theta_error)< 0.2:
                 theta_error = 0.0
             total_error = total_error+theta_error*del_time
             # if total_error> 6.28:
             #     total_error = 6.28
             # elif(total_error<-3.14):
             #     total_error = -3.14

             delta_error = (theta_error - prev_error)/del_time

             kp=1
             ki=1
             kd=1
             # kpc=1
             # kic=1
             # kdc=1
             self.force = kp*theta_error + (ki)*total_error + (kd)*delta_error
             # if self.force> 3.14:
             #     self.force = 3.14
             # elif(self.force<-3.14):
             #     self.force = -3.14
             self.ctrl_force_pub.publish(self.force)
             prev_error = theta_error
             last_time = curr_time



if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
    rospy.init_node("Pendulum", anonymous=True)
    rospy.loginfo("[Pendulum]: Node Started")
    Pendulum()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[StreamViewer]: Shutting down node")







#!/usr/bin/env python3

import rospy
import rospkg
import pygame as pg
import sys
from math import sin, cos, pi
from inverted_pendulum_sim.msg import CurrentState
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest

'''
pygame initialization and parameter settings
'''

class Pendulum:
    """
    This class simulates the inverted pendulum.
    """

    def __init__(self):
        """
        @brief: Constructor - Initializes the inverted pendulum simulation.
        """
        self.x = 0
        self.x_dot = 0
        self.x_d_dot = 0
        self.theta = 3.14
        self.theta_dot = 0
        self.theta_d_dot = 0
        self.length = 300
        self.cart_m = 0.5
        self.pendulum_m = 2
        self.g = -9.8
        self.start = rospy.get_time()
        self.force = 0
        self.cur_theta =0.0
        self.current_state_sub = None
        self.ctrl_force_pub = None
        now = rospy.get_rostime()
        self.init_pubs_subs()
        base_set = rospy.ServiceProxy( '/inverted_pendulum/set_params', SetParams )
        base_set(2.0, 300, 0.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0)


        '''
        starting main loop
        '''
        self.main_loop()

    def init_pubs_subs(self):
    
        self.ctrl_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce , queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState,
                                               self.current_state_callback)


    def current_state_callback(self, msg):
   
        # self.theta = msg.curr_theta
         self.theta_dot = msg.curr_theta_dot
        # self.theta_d_dot = msg.curr_theta_dot_dot
        # self.x = msg.curr_x
        # self.x_dot = msg.curr_x_dot
        # self.x_d_dot = msg.curr_x_dot_dot
        self.cur_theta = msg.curr_theta
        self.cur_x = msg.curr_x


    def main_loop(self):
        """
        @brief: Main loop of the simulation.
        :return: none
        """
        last_time = 0.0
        prev_error = 0.0
        sample_time = 1
        total_error =0
        delta_error=0
        while not rospy.is_shutdown():

            curr_time = rospy.get_time()*1000
            del_time = curr_time - last_time
            

            theta_error = self.theta - self.cur_theta
            if abs(theta_error)< 0.2:
                theta_error = 0.0
            total_error = total_error+ theta_error*del_time
            # if total_error> 3.14:
            #     total_error = 3.14
            # elif(total_error<-3.14):
            #     total_error = -3.14

            delta_error = (theta_error - prev_error)/del_time

            kp=200
            ki=1
            kd=1
            # kpc=1
            # kic=1
            # kdc=1
            self.force = kp*self.cur_theta + (ki)*self.theta_dot + (kd)*delta_error
            self.ctrl_force_pub.publish(self.force)
            rospy.loginfo(self.force)
            prev_error = theta_error
            last_time = curr_time



if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
    rospy.init_node("Pendulum", anonymous=True)
    rospy.loginfo("[Pendulum]: Node Started")
    Pendulum()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[StreamViewer]: Shutting down node")










#!/usr/bin/env python3

import rospy
import rospkg
import pygame as pg
import sys
from math import sin, cos, pi
from inverted_pendulum_sim.msg import CurrentState
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest

'''
pygame initialization and parameter settings
'''

class Pendulum:
    """
    This class simulates the inverted pendulum.
    """

    def __init__(self):
        """
        @brief: Constructor - Initializes the inverted pendulum simulation.
        """
        self.x = 0
        self.x_dot = 0
        self.x_d_dot = 0
        self.theta = 3.14
        self.theta_dot = 0
        self.theta_d_dot = 0
        self.length = 300
        self.cart_m = 0.5
        self.pendulum_m = 2
        self.g = -9.8
        self.start = rospy.get_time()
        self.force = 0
        self.cur_theta =0.0
        self.current_state_sub = None
        self.ctrl_force_pub = None
        now = rospy.get_rostime()
        self.init_pubs_subs()
        base_set = rospy.ServiceProxy( '/inverted_pendulum/set_params', SetParams )
        base_set(2.0, 300, 0.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0)


        '''
        starting main loop
        '''
        self.main_loop()

    def init_pubs_subs(self):
    
        self.ctrl_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce , queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState,
                                               self.current_state_callback)


    def current_state_callback(self, msg):
   
        # self.theta = msg.curr_theta
        # self.theta_dot = msg.curr_theta_dot
        # self.theta_d_dot = msg.curr_theta_dot_dot
        # self.x = msg.curr_x
        # self.x_dot = msg.curr_x_dot
        # self.x_d_dot = msg.curr_x_dot_dot
        self.cur_theta = msg.curr_theta
        self.cur_x = msg.curr_x


    def main_loop(self):
        """
        @brief: Main loop of the simulation.
        :return: none
        """

        last_time = 0.0
        prev_error = 0.0
        sample_time = 1
        total_error =0
        delta_error=0
        while not rospy.is_shutdown():

            curr_time = rospy.get_time()*1000
            del_time = curr_time - last_time
            

            theta_error = self.theta - self.cur_theta
            if abs(theta_error)< 0.2:
                theta_error = 0.0
            total_error = total_error+ theta_error
            # if total_error> 3.14:
            #     total_error = 3.14
            # elif(total_error<-3.14):
            #     total_error = -3.14

            delta_error = (theta_error - prev_error)

            kp=2500
            ki=1
            kd=100
            # kpc=1
            # kic=1
            # kdc=1
            self.force = kp*theta_error + (ki)*total_error + (kd)*delta_error
            self.ctrl_force_pub.publish(self.force)
            rospy.loginfo(self.force)
            prev_error = theta_error
            last_time = curr_time


if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
    rospy.init_node("Pendulum", anonymous=True)
    rospy.loginfo("[Pendulum]: Node Started")
    Pendulum()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[StreamViewer]: Shutting down node")



