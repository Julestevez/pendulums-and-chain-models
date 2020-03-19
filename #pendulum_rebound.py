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
tmax     = 50
dt       = 0.05
theta    = -0.30
thetadot = 0.8
barrera=0.7
x1=math.sin(theta)

def thetadotdot(theta, thetadot, time):

    pi = 3.14159265359
    # The diff eq!
    thetadotdot = -grav/length*math.sin(theta) - damping*thetadot 
    return thetadotdot

while time<tmax:
    if (x1+0.15)>barrera:
       thetadot=-thetadot

    else:
        thetadot = thetadot + thetadotdot(theta, thetadot, time)*dt
    
    theta    = theta + thetadot*dt
    x0,y0=0,length+0.02
    x1=length*math.sin(theta)
    y1=length-length*math.cos(theta)
    
    time     = time + dt
    #plt.plot(time,theta,'or')
    fig = plt.figure()
    plt.axis([-1.5,1.5,0,2.6])
    plt.plot([x0,x1],[y0,y1])
    plt.plot([x1],[y1],'ro',markersize=20)
    plt.axvline(x=barrera,linewidth=4, color='g')
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