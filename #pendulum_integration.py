#pendulum integration
import matplotlib.pyplot as plt
import math


## Physical constants.
mass    = 1.0 # mass of pendulum
length  = 0.25*9.8 # length of pendulum
damping = 0.1 # damping coefficient
grav    = 9.8 # magnitude of graviational field

#now comes the pendulum
## Initialize run.
time     = 0.0
tmax     = 300
dt       = 0.05
theta    = 0.05
thetadot = 0.8

def thetadotdot(theta, thetadot, time):

    pi = 3.14159265359
    # The diff eq!
    thetadotdot = -grav/length*math.sin(theta) - damping*thetadot 
    return thetadotdot


# Bar lifting
x0,y0=0,0
x1,y1=-length,0

while time<tmax:
    while y0 < length:
        y0=y0+0.1
        x0=0
        y1=0
        x1=x1+0.1
        plt.axis([-1.5,1.5,0,2.6])
        plt.plot([x0,x1],[y0,y1])
        plt.plot([x1],[y1],'ro',markersize=20)
        plt.pause(0.05)
        plt.cla()
    


    thetadot = thetadot + thetadotdot(theta, thetadot, time)*dt
    theta    = theta + thetadot*dt
    x0,y0=0,length+0.02
    x1=length*math.sin(theta)
    y1=length-length*math.cos(theta)
    time     = time + dt
    #plt.plot(time,theta,'or')
    plt.axis([-1.5,1.5,0,2.6])
    plt.plot([x0,x1],[y0,y1])
    plt.plot([x1],[y1],'ro',markersize=20)
    plt.pause(0.05)
    plt.cla()
plt.show()



#gdisplay(x=200,y=400)
#fdvstime = gcurve(color=color.white)





# while time < tmax:
#     thetadot = thetadot + thetadotdot(theta, thetadot, time)*dt
#     theta    = theta + thetadot*dt
#     time     = time + dt
#     plt.plot(time,theta,'or')
#     plt.pause(0.05)
# plt.show()