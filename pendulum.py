import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

guiFlag = False

dt = 1/240 # pybullet simulation step
th0 = 0.1  # starting position (radian)
thd = 1.0  # desired position (radian)
kp = 40.0
kd = 12.0
g = 10     # m/s^2
L = 0.8    # m
m = 1      # kg
moveTime = 2.0 # seconds
maxTime = 4.0


def fifth_order_traj(t):
    # fifth-order polynomial from Modern Robotics 9
    if t > moveTime:
        t = moveTime

    r = t / moveTime
    s = 10*r**3 - 15*r**4 + 6*r**5
    ds = (30*r**2 - 60*r**3 + 30*r**4) / moveTime
    dds = (60*r - 180*r**2 + 120*r**3) / (moveTime**2)

    th_ref = th0 + s * (thd - th0)
    vel_ref = ds * (thd - th0)
    acc_ref = dds * (thd - th0)
    return th_ref, vel_ref, acc_ref


physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf.xml", useFixedBase=True)

# get rid of all the default damping forces
# think of it as imagined "air drag"
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.resetJointState(boxId, 1, th0, 0)

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
logThetaSim = np.zeros(sz)
logVelSim = np.zeros(sz)
logTauSim = np.zeros(sz)
logThetaDes = np.zeros(sz)
logVelDes = np.zeros(sz)
idx = 0

for t in logTime:
    th = p.getJointState(boxId, 1)[0]
    vel = p.getJointState(boxId, 1)[1]

    th_ref, vel_ref, acc_ref = fifth_order_traj(t)

    logThetaSim[idx] = th
    logVelSim[idx] = vel
    logThetaDes[idx] = th_ref
    logVelDes[idx] = vel_ref

    e = th - th_ref
    de = vel - vel_ref
    u = -kp*e - kd*de
    tau = (m*L*L) * (g/L*np.sin(th) + u)
    logTauSim[idx] = tau

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=tau, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()

    idx += 1
    if guiFlag:
        time.sleep(dt)
p.disconnect()

plt.subplot(3,1,1)
plt.plot(logTime, logThetaSim, 'b', label="Sim Pos")
plt.plot(logTime, logThetaDes, 'r--', label="Desired Pos")
plt.axvline(moveTime, color='k', linestyle=':', label="T")
plt.grid(True)
plt.legend()

plt.subplot(3,1,2)
plt.plot(logTime, logVelSim, 'b', label="Sim Vel")
plt.plot(logTime, logVelDes, 'r--', label="Desired Vel")
plt.grid(True)
plt.legend()

plt.subplot(3,1,3)
plt.plot(logTime, logTauSim, 'b', label="Tau")
plt.grid(True)
plt.legend()
plt.show()

# dt = 0.1
# dx = f(x,t) t = [0, 0.1, 0.2]
# x(t)
# dx/dt = f(x,t)
# dx = dt * f(x,t)
# x[n+1] - x[n] = dt * f(x,t)
# x[n+1] = x[n] + dt*f(x,t) # Euler method

# lim(dx/dt) dt -> 0

# ddth = -g/L * sin(th)
# mL^2*ddth + mgLsin(th) = tau
# ddth = - g/Lsin(th) + tau/(mL^2)
# tau = (mL^2)g/Lsin(th) + u(t) -> ddth = -g/Lsin(th) + ((mL^2)g/Lsin(th) + u(t))/(mL^2)
# ddth = -g/Lsin(th) + g/Lsin(th) + u(t)
# ddth = u(t)
# ddth = kp(th-thd)
# Feedback linearization

# dx = ax
# dx(t) = f(x,t)
# dth = w
# dw = -g/Lsin(th)

# dth = w
# dw = -g/L*th
# X = (th, w)
# dX = A*X = [0 1; -g/L 0]
# X = e^(A*t)

# dx/dt = ax
# dx / x = a dt
# ln(x) = at + C
# x = e^(at)

# LTI
# dX = A*X + B*tau
# tau = K*X
# dx = A*X + B*K*X = (A+BK)X

# Forward Kinematics
# x = -L1*sin(th1) - L2*sin(th1+th2)
# z = H - L1*cos(th1) - L2*cos(th1+th2)

# dx = -L1*cos(th1)*dth1 - L2*cos(th1+th2)*(dth1+dth2)
# dz = L1*sin(th1)*dth1 + L2*sin(th1+th2)*(dth1+dth2)

# dx = (-L1*cos(th1) - L2*cos(th1+th2))*dth1 - L2*cos(th1+th2) * dth2
# dz = (L1*sin(th1) + L2*sin(th1+th2))*dth1 + L2*sin(th1+th2) * dth2
# X = (x,z)'
# Th = (th1, th2)'
# dX = J(Th) * dTh
# dTh = inv(J) * dX
# dX = k(Xd - X)
