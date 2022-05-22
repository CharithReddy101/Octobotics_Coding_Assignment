#!/usr/bin/env python3

'''
This node sets initial paramters for the inverted pendulum.
'''

import rospy
import sys
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest


def setBase():
    """
    @brief: Sets inital parameter to the inverted pendulum.
    pendulum weight = 2kg, 
    cart weight = 0.5kg, 
    pendulum length = 300units, 
    cart position= in the centre, 
    pendulum orientation = vertical down 
    :return: none
    """
    pendulum_m = 2
    length = 300
    cart_m = 0.5
    g = -9.8
    force = 0

    theta_initial =0.0
    theta_dot =0.0
    theta_d_dot =0.0
    cur_theta =0.0

    x =0.0
    x_dot =0.0
    x_d_dot =0.0

    base_set = rospy.ServiceProxy( '/inverted_pendulum/set_params', SetParams )
    base_set(pendulum_m, length, cart_m, theta_initial, theta_dot, theta_d_dot, x, x_dot, x_d_dot)


if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
 
    setBase()


