#!/usr/bin/env python3

from control import *
import numpy as np
import matplotlib.pyplot as plt #
class test:
    """
    This class simulates the inverted pendulum.
    """

    def __init__(self):

      
        self.main_loop()


    def main_loop(self):

        M = 0.5
        m = 2.0
        b = 0.1
        I = 0.06
        g = 9.8
        l = 0.3
        kp=20
        kd=0.1
        ki=1
        q = (M+m)*(I+m*l**2)-(m*l)**2
        s = tf('s')
        P_pend = (m*l*s/q)/(s**3 + (b*(I + m*l**2))*s**2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q)
        C = kp + ki/s +kd*s 
        P_cart = (((I+m*l**2)/q)*s**2 - (m*g*l/q))/(s**4 + (b*(I + m*l**2))*s**3/q - ((M + m)*m*g*l)*s**2/q - b*m*g*l*s/q);
        T2 = feedback(1,P_pend*C)*P_cart;
        print(T2)  
        t_step = np.linspace(0,3,200)
        t_step,y_step = step_response(T2, t_step)   
        (t, y, x) = forced_response(T2, t, u, x0) 
        print (y_step)
        print (t_step)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        line, = ax.plot(t_step, y_step)
        plt.xlabel('time (s)')
        plt.ylabel('step response')
        plt.show()
                    
            
            


if __name__ == '__main__':
    """
    @brief: Main function.
    :return: none
    """
   
    test()

