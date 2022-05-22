#!/usr/bin/env python3

'''
This node controls the cart and balances the inverted pendulum.
'''

import rospy
import rospkg
import pygame as pg
import sys
from math import sin, cos, pi
from inverted_pendulum_sim.msg import CurrentState
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest
from simple_pid import PID


class BalancePendulum:
    """
    This class balances the inverted pendulum.
    """

    def __init__(self):
        """
        @brief: Constructor - Initializes the parameters required to control cart.
        """
        self.pendulum_m = 2
        self.length = 300
        self.cart_m = 0.5
        self.g = -9.8
        self.force = 0

        self.theta = 3.14
        self.theta_initial =3.0
        self.theta_dot =0.0
        self.theta_d_dot =0.0
        self.cur_theta =0.0

        self.cur_x =0.0
        self.x =0.0
        self.x_dot =0.0
        self.x_d_dot =0.0

        self.start = rospy.get_time()
        self.current_state_sub = None
        self.ctrl_force_pub = None
        self.base_set = None

        self.pid = PID(500, 2, 20, setpoint=3.14)
        self.pid.output_limits = (-1000, 1000)
        self.init_pubs_subs()


        '''
        starting main loop
        '''
        self.main_loop()

    def init_pubs_subs(self):
        """
        @brief: Initializes the publishers, subscribers and services.
        :return: none
        """
        self.base_set = rospy.ServiceProxy( '/inverted_pendulum/set_params', SetParams )
        self.ctrl_force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce , queue_size=10)
        self.current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState,
                                               self.current_state_callback)

    def current_state_callback(self, msg):
        """
        @brief: Callback for the subscriber to set the current state of the system.
        :return res: None
        """

        self.cur_theta = msg.curr_theta
        if self.cur_theta < 0.0:
            self.cur_theta = self.theta + (self.theta+self.cur_theta)
        self.cur_x = msg.curr_x


    def main_loop(self):
        """
        @brief: Main loop of the simulation.
        :return: none
        """
        self.base_set(self.pendulum_m, self.length, self.cart_m, self.theta_initial, self.theta_dot, self.theta_d_dot, self.x, self.x_dot, self.x_d_dot)
        while not rospy.is_shutdown():
            f1 = self.pid(self.cur_theta)
            self.force = f1
            self.ctrl_force_pub.publish(self.force)
            #rospy.loginfo(self.force)
          

if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
    rospy.init_node("BalancePendulum", anonymous=True)
    rospy.loginfo("[BalancePendulum]: Node Started")
    BalancePendulum()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[StreamViewer]: Shutting down node")



