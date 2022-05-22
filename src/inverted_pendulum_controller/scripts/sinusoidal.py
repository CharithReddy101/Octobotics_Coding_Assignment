#!/usr/bin/env python3

"""
This node simulates the sinusoidal force with different amplitudes and frequencies.
"""
import rospy
import rospkg
import pygame as pg
import sys
from math import sin, cos, pi
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest

class sineWave:
    """
    This class simulates the sinusoidal force with different amplitudes and frequencies.

    """

    def __init__(self):
        """
        @brief: Constructor - Initializes the base parameters of the simulation.
        """
        self.init_pubs_subs()

        '''
        starting main loop
        '''
        self.main_loop()

    def init_pubs_subs(self):
        """
        @brief: Initializes the publishers and services .
        :return: none
        """

        self.force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
        self.base_set = rospy.ServiceProxy( '/inverted_pendulum/set_params', SetParams )
        self.base_set(2.0, 300, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  


    def main_loop(self):
        """
        @brief: Main loop of the simulation.
        :return: none
        """

        self.t=0

        while not rospy.is_shutdown():


            if self.t <3:              
                self.amplitude = 50
                self.freq = 2
                self.force = 0  
                #self.base_set(2.0, 300, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  


            elif self.t <6:
                self.amplitude = 80
                self.freq = 2
                self.force = 0
                #self.base_set(2.0, 300, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  

            elif self.t >6:
                self.amplitude = 150
                self.freq = 4
                self.force = 0  

            
            self.force = self.amplitude*sin(2*pi*self.freq*self.t)
            
            self.force_pub.publish(int(self.force))
            rospy.loginfo(int(self.amplitude))
            #self.t=self.t+0.05
            self.t=self.t+0.2
            rospy.sleep(0.2)


if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
    rospy.init_node("SinusoidalForce", anonymous=True)
    rospy.loginfo("[SinusoidalForce]: Node Started")
    sineWave()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[StreamViewer]: Shutting down node")
