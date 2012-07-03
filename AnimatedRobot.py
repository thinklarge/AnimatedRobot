
# -----------------
# USER INSTRUCTIONS
#
# Write a function in the class robot called move()
#
# that takes self and a motion vector (this
# motion vector contains a steering* angle and a
# distance) as input and returns an instance of the class
# robot with the appropriate x, y, and orientation
# for the given motion.
#
# *steering is defined in the video
# which accompanies this problem.
#
# For now, please do NOT add noise to your move function.
#
# Please do not modify anything except where indicated
# below.
#
# There are test cases which you are free to use at the
# bottom. If you uncomment them for testing, make sure you
# re-comment them before you submit.
import numpy as np

from pylab import *
from math import *
import random

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
# --------
# 
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"
max_steering_angle = pi/4 # You don't need to use this value, but it is good to keep in mind the limitations of a real car.


# ------------------------------------------------
# 
# this is the robot class
#

class robot:

    # --------

    # init: 
    #    creates robot and initializes location/orientation 
    #

    def __init__(self, length = 10.0):
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero
        
   
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
    # --------
    # set: 
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise (ValueError, 'Orientation must be in [0..2pi]')
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
        
        


    # --------
    # set_noise: 
    #	sets the noise parameters
    #

    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
        
   
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################

    # --------
    # move:
    #   move along a section of a circular path according to motion
    #
    
    def move(self, motion): # Do not change the name of this function
        x = self.x
        y = self.y
        theta = self.orientation
        phi = motion[0]
        d = motion[1]
        L = self.length
        
        result = robot()
        result = self

        
        beta = d/L * tan(phi)
        
        if (abs(beta) < 0.001):
            result.x = x + d*cos(theta)
            result.y = y + d*sin(theta)
            result.orientation = theta
        else:    
            R = (d / beta)     
            cx = x - sin(theta)*R
            cy = y + cos(theta)*R

            
            result.x = cx + R*sin(theta+beta)
            result.y = cy - R*cos(theta+beta)
            result.orientation = (theta+beta)%(2*pi)
        
        # ADD CODE HERE
        
        return (result) # make sure your move function returns an instance
                      # of the robot class with the correct coordinates.
                      
    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################
class RobotPositions:
    def __init__(self, robo = robot()):

        # Body - Wheel Ratio
        lengratio = 1.0

        leng = robo.length        
        
        # Body Buffer info
        self.bufferfront = 3.0
        self.bufferrear = 2.0
        self.bufferside = 1.0

        
        # Wheels
        self.lengwf = (leng*lengratio)
        self.lengwb = leng*(1-lengratio)
        self.widw = leng/4 
        self.radwf = math.sqrt(self.widw**2 + self.lengwf**2)
        self.radwb = math.sqrt(self.widw**2 + self.lengwb**2)
        self.betawf = math.atan2(self.widw,self.lengwf)
        self.betawb = math.atan2(self.widw, self.lengwb)
       
        #Car Body
        self.lengf = (leng*lengratio + self.bufferfront)
        self.lengb = (leng*(1-lengratio) + self.bufferrear)
        self.wid = (leng/4 + self.bufferrear)
        self.radf = math.sqrt(self.wid**2 + self.lengf**2)
        self.radb = math.sqrt(self.wid**2 + self.lengb**2)
        self.betaf = math.atan2(self.wid,self.lengf)
        self.betab = math.atan2(self.wid, self.lengb)
        
        self.linesx = [[]]
        self.linesx.append([])
        self.linesx.append([])
        self.linesx.append([])
        self.linesx.append([])
        
        self.linesy = [[]]
        self.linesy.append([])
        self.linesy.append([])
        self.linesy.append([])
        self.linesy.append([])
        
    def wheelsnapshot(self, inrobo, steering):
        cx = inrobo.x
        cy = inrobo.y
        rx= []
        ry = []
        lw = 1
        o = inrobo.orientation
        test = [self.betawf + o, -self.betawf + o, pi + self.betawb + o, pi -self.betawb + o]

        for i in range(4):
            wx = []
            wy = []
            if (i < 2):
                ti = test[i]
                ti2 = test[0] + steering
                rx=(cx + self.radwf*cos(ti))
                ry=(cy + self.radwf*sin(ti))
                wx.append(rx + lw * cos(ti2))
                wx.append(rx + lw * cos(ti2 + pi))
                wy.append(ry + lw * sin(ti2))
                wy.append(ry + lw * sin(ti2 + pi))

            else:
                ti = test[i]
                ti2 = test[i] + pi/2
                rx=(cx + self.radwb*cos(ti))
                ry=(cy + self.radwb*sin(ti))
                wx.append(rx + lw * cos(ti2))
                wx.append(rx + lw * cos(ti2 + pi))
                wy.append(ry + lw * sin(ti2))
                wy.append(ry + lw * sin(ti2 + pi))
            self.linesx[i].append(wx) 
            self.linesy[i].append(wy)
        roboart = RobotPositions()
        roboart = self
        return(roboart)
    
        
    def snapshot(self, inrobo, steering):
        cx = inrobo.x
        cy = inrobo.y

        rx= []
        ry = []
        test = [self.betaf, -self.betaf, pi + self.betab, pi -self.betab]
        for i in range(4):
            if (i < (len(test)/2)):
                rx.append(cx + self.radf*cos(inrobo.orientation + test[i]))
                ry.append(cy + self.radf*sin(inrobo.orientation + test[i]))
            else:
                rx.append(cx + self.radb*cos(inrobo.orientation + test[i]))
                ry.append(cy + self.radb*sin(inrobo.orientation + test[i]))
        roboart = RobotPositions()
        roboart = self.wheelsnapshot(inrobo, steering)
        rx.append(rx[0])
        ry.append(ry[0])
        roboart.linesx[4].append(rx)
        roboart.linesy[4].append(ry)
        return roboart      

    def plotmotion(self):
        lx = self.linesx
        ly = self.linesy
        
        fig = plt.figure()
        ax = fig.add_subplot(111, autoscale_on=False, xlim=(-150, 300), ylim=(-150, 300))
        ax.grid()
        linefr, = ax.plot([], [], lw=1)
        linefl, = ax.plot([], [], lw=1)
        linebr, = ax.plot([], [], lw=1)
        linebl, = ax.plot([], [], lw=1)
        linebod, = ax.plot([], [], lw=1)
        
        lxperm = []
        lyperm = []
        for i in range(len(lx[4])):
            lxperm.append(lx[4][i][0])
            lyperm.append(ly[4][i][0])
        plt.plot(lxperm,lyperm, lw=1)
        
        def init():
            linefr.set_data([], [])
            linefl.set_data([], [])
            linebr.set_data([], [])
            linebl.set_data([], [])
            linebod.set_data([], [])
            return linefr, linefl, linebr, linebl, linebod
        
        def animate(i):
            linefr.set_data(lx[0][i], ly[0][i])
            linefl.set_data(lx[1][i], ly[1][i])
            linebr.set_data(lx[2][i], ly[2][i])
            linebl.set_data(lx[3][i], ly[3][i])
            linebod.set_data(lx[4][i], ly[4][i])
        
            return linefr, linefl, linebr, linebl, linebod
            
        for i in range(1, T):
            ani = animation.FuncAnimation(fig, animate, np.arange(1, T), interval=1)    
        
        #ani.save('test_sub.mp4')
        
        plt.show()

## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.

## --------
## TEST CASE:
## 
## 1) The following code should print:
##       Robot:     [x=0.0 y=0.0 orient=0.0]
##       Robot:     [x=10.0 y=0.0 orient=0.0]
##       Robot:     [x=19.861 y=1.4333 orient=0.2886]
##       Robot:     [x=39.034 y=7.1270 orient=0.2886]


##length = 20.
##bearing_noise  = 0.0
##steering_noise = 0.0
##distance_noise = 0.0

##myrobot = robot(length)
##myrobot.set(0.0, 0.0, 0.0)
##myrobot.set_noise(bearing_noise, steering_noise, distance_noise)

##motions = [[0.0, 10.0], [pi / 6.0, 10], [0.0, 20.0]]

##T = len(motions)

##print ('Robot:    ', myrobot)
##for t in range(T):
    ##myrobot = myrobot.move(motions[t])
    ##print ('Robot:    ', myrobot)



## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.

    
## 2) The following code should print:
##      Robot:     [x=0.0 y=0.0 orient=0.0]
##      Robot:     [x=9.9828 y=0.5063 orient=0.1013]
##      Robot:     [x=19.863 y=2.0201 orient=0.2027]
##      Robot:     [x=29.539 y=4.5259 orient=0.3040]
##      Robot:     [x=38.913 y=7.9979 orient=0.4054]
##      Robot:     [x=47.887 y=12.400 orient=0.5067]
##      Robot:     [x=56.369 y=17.688 orient=0.6081]
##      Robot:     [x=64.273 y=23.807 orient=0.7094]
##      Robot:     [x=71.517 y=30.695 orient=0.8108]
##      Robot:     [x=78.027 y=38.280 orient=0.9121]
##      Robot:     [x=83.736 y=46.485 orient=1.0135]
##
##
length = 20.
bearing_noise  = 0.0
steering_noise = 0.0
distance_noise = 0.0
myrobot = robot(length)
robotart = RobotPositions()
myrobot.set(0.0, 0.0, 0.0)
myrobot.set_noise(bearing_noise, steering_noise, distance_noise)



motions = [[0.4, 1.]]
for row in range(3000):
    move = .2 + .005 * row
    motions.append([move,1.])

#for row in range(500):
#    move = -.2 - .001 * row
#    motions.append([move,1.])

T = len(motions)


x = []
y = []
for t in range(T):
    myrobot = myrobot.move(motions[t])
    x.append(myrobot.x)
    y.append(myrobot.y)
    if not (t%2):    #this limits the number of snapshots we take.
        robotart = robotart.snapshot(myrobot, motions[t][0])
    #print ('Robot:    ', myrobot)

robotart.plotmotion()
## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out. Our testing program provides its own code for testing your
## move function with randomized motion data.


