#!/bin/python3
from numpy import linspace, sin, pi
from scipy.interpolate import BPoly, CubicSpline

xi = [0, pi/2, pi]
xnew = linspace(0, pi, 50)

yi = sin(xi)
ynew = sin(xnew)
yder = [[0,0,0,0,0],[1,0,0,0,0],[0,0,0,0,0]]

cubic = CubicSpline(xi, yi)
print(xi)
print(yder)
bpoly = BPoly.from_derivatives(xi, yder, orders=9)

y_bpoly = bpoly(xnew)
y_cubic = cubic(xnew)

import matplotlib.pyplot as plt

figure, axis = plt.subplots(5)

# axis[0].plot(xnew, y_bpoly, '-g', xnew, ynew, '--c', xnew, y_cubic, '-.m', xi, yi, '*r')
# axis[0].legend(['BPoly', 'True', 'Cubic', 'Points'])
# axis[0].grid()

axis[0].plot(xnew, y_bpoly, '-g', xi, yi, '*r')
axis[0].legend(['BPoly', 'Points'])
axis[0].grid()

vel_spl = bpoly.derivative()
y_vels = vel_spl(xnew)
axis[1].plot(xnew,y_vels)

acc_spl = vel_spl.derivative()
acc_vels = acc_spl(xnew)
axis[2].plot(xnew,acc_vels)

jrk_spl = acc_spl.derivative()
jrk_vals = jrk_spl(xnew)
axis[3].plot(xnew,jrk_vals)

snap_spl = jrk_spl.derivative()
snap_vals = snap_spl(xnew)
axis[4].plot(xnew,snap_vals)



plt.show()