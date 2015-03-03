#!/usr/bin/env python
import roslib; roslib.load_manifest('jsk_quadcopter')
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16

import sys, select, termios, tty

from numpy import *
import pylab as plt
 
theta = linspace(0, pi, 200)

h = 0.5
d = 0.3
l = 0.5

#x1 = h/2 * sin(theta) + sqrt(2) *d * sin(pi/4 + theta/2)
#y1 = h/2 * cos(theta) + sqrt(2) *d * cos(pi/4 + theta/2)

a = (h + 2 *d)/2
b = h/2 + (sqrt(2) -1) * d

theta2 = linspace(-pi/2, pi/2, 200)
x2 = b * cos(theta2) + d
y2 = a * sin(theta2)

x1 = h/2 * cos(theta2) + sqrt(2) *d * cos(theta2/2)
y1 = h/2 * sin(theta2) + sqrt(2) *d * sin(theta2/2)


plt.plot(x1, y1, color="k", marker="o")
plt.plot(x2, y2, color="b", marker="o")

psi = linspace(0, pi/4, 100)
 
y3_tmp = (2*l - d/2) * sin(psi) 
y3 = y3_tmp + sqrt(2)* d * sin(pi/4 + psi) + h / 2
x3 = - y3_tmp * tan(psi) + sqrt(2)* d * cos(pi/4 + psi)
y5 = -y3

y4 = (2*l - d/2) * psi + d *( psi + 1) + h / 2
x4 = - (2*l - d/2) * psi  * psi + d * (1 -psi)
y6 = -y4


l_x = [-1, 0, 0, -1]
l_y = [h/2, h/2, -h/2, -h/2]

x_tmp = linspace(-1, d , 100)
y_tmp = d + (2*l - d/2 + d) * (sqrt(d*d/(4*(2*l - d/2)*(2*l - d/2)) + (d-x_tmp)/(2*l - d/2)) - d/2/(2*l - d/2)) + h/2
#y_tmp =  sqrt(-x_tmp*(2*l- d/2 + d) + d*((2*l- d/2) + 2* d)) + h /2

plt.plot(l_x, l_y, color="r", marker="*")

#plt.plot(x3, y3, color="k", marker="o")
#plt.plot(x3, y5, color="k", marker="o")
plt.plot(x4, y4, color="b", marker="o")
plt.plot(x4, y6, color="b", marker="o")

#plt.plot(x_tmp, y_tmp, color="g", marker="o")

plt.show()
