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
theta    = 0
thetadot = 0
accel_x  = 0


def thetadotdot(theta, thetadot, time):

    pi = 3.14159265359
    # The diff eq!
    thetadotdot = accel_x/length - grav/length*math.sin(theta) - damping*thetadot
    return thetadotdot


# Bar lifting
x0,y0=0,0
x1,y1=-length,0
x0_vel=0
barrera=4

while time<tmax:
    accel_x=math.sin(2*time)
    x0_vel= x0_vel + dt*accel_x
    x0 = x0 + x0_vel*dt
    
    #bounce on the wall
    if x1+0.30 > barrera:
        thetadot=-thetadot
    else:
        thetadot = thetadot + thetadotdot(theta, thetadot, time)*dt
    
    theta    = theta + thetadot*dt
    x1=x0-length*math.sin(theta)
    y1=y0-length*math.cos(theta)
        
    time     = time + dt
    #plt.plot(time,theta,'or')
    plt.axis([-1.5,6,-3,3])
    plt.plot([x0,x1],[y0,y1])
    plt.plot([x1],[y1],'ro',markersize=20)
    plt.axvline(x=4,linewidth=4,color="green")
    plt.pause(0.05)
    plt.cla()
plt.show()
