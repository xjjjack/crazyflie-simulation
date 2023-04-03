#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 
# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""


from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

from math import cos, sin, degrees, radians 

import sys
# Change this path to your crazyflie-firmware folder
sys.path.append('../../../../../c/crazyflie-firmware-experimental/build')
import cffirmware

import numpy as np
import matplotlib.pyplot as plt

robot = Robot()

timestep = int(robot.getBasicTimeStep())

## Initialize motors
m1_motor = robot.getDevice("m1_motor")
m1_motor.setPosition(float('inf'))
m1_motor.setVelocity(-1)
m2_motor = robot.getDevice("m2_motor")
m2_motor.setPosition(float('inf'))
m2_motor.setVelocity(1)
m3_motor = robot.getDevice("m3_motor")
m3_motor.setPosition(float('inf'))
m3_motor.setVelocity(-1)
m4_motor = robot.getDevice("m4_motor")
m4_motor.setPosition(float('inf'))
m4_motor.setVelocity(1)

## Initialize Sensors
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
Keyboard().enable(timestep)
keyboard = Keyboard()
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
camera = robot.getDevice("camera")
camera.enable(timestep)
range_front = robot.getDevice("range_front")
range_front.enable(timestep)
range_left = robot.getDevice("range_left")
range_left.enable(timestep)
range_back = robot.getDevice("range_back")
range_back.enable(timestep)
range_right = robot.getDevice("range_right")
range_right.enable(timestep)
## Initialize variables
pastXGlobal = 0
pastYGlobal = 0
pastZGlobal = 0

past_time = robot.getTime()

cffirmware.controllerPidInit()
#mellinger_control = cffirmware.controllerMellinger_t()
#cffirmware.controllerMellingerInit(mellinger_control)
print('Take off!')

max_time = 10
plt.axis([0,max_time,0,35000])
plt.ion()
plt.show()
x = []
y1=[]
y2 = []

pastRoll = 0
pastPitch = 0
pastYaw = 0
# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time

    print(robot.getTime())

    ## Get measurements
    roll = imu.getRollPitchYaw()[0]
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    roll_rate = gyro.getValues()[0]
    pitch_rate = gyro.getValues()[1]
    yaw_rate = gyro.getValues()[2]
    # atlernative rate calculation
    #roll_rate = (roll - pastRoll)/dt
    #pitch_rate = (pitch - pastPitch)/dt
    #yaw_rate = (yaw - pastYaw)/dt
    xGlobal = gps.getValues()[0]
    vxGlobal = (xGlobal - pastXGlobal)/dt
    yGlobal = gps.getValues()[1]
    vyGlobal = (yGlobal - pastYGlobal)/dt
    zGlobal = gps.getValues()[2]
    vzGlobal = (zGlobal - pastZGlobal)/dt


    ## Put measurement in state estimate
    # TODO replace these with a EKF python binding
    state = cffirmware.state_t()
    state.attitude.roll = degrees(roll)
    state.attitude.pitch = -degrees(pitch)
    state.attitude.yaw = degrees(yaw)
    state.position.x = xGlobal
    state.position.y = yGlobal
    state.position.z = zGlobal
    state.velocity.x = vxGlobal
    state.velocity.y = vyGlobal
    state.velocity.z = vzGlobal
    
    # Put gyro in sensor data
    sensors = cffirmware.sensorData_t()
    sensors.gyro.x = degrees(roll_rate)
    sensors.gyro.y = degrees(pitch_rate)
    sensors.gyro.z = degrees(yaw_rate)

    # keyboard input
    forwardDesired = 0
    sidewaysDesired = 0
    yawDesired = 0

    key = keyboard.getKey()
    while key>0:
        if key == Keyboard.UP:
            forwardDesired = 0.5
        elif key == Keyboard.DOWN:
            forwardDesired = -0.5
        elif key == Keyboard.RIGHT:
            sidewaysDesired = -0.5
        elif key == Keyboard.LEFT:
            sidewaysDesired = 0.5
        elif key == ord('Q'):
            yawDesired = 1
        elif key == ord('E'):
            yawDesired = -1

        key = keyboard.getKey()

    ## Example how to get sensor data
    # range_front_value = range_front.getValue();
    # cameraData = camera.getImage()

    ## Fill in Setpoints
    setpoint = cffirmware.setpoint_t()
    setpoint.mode.z = cffirmware.modeAbs
    setpoint.position.z = 1.0
    setpoint.mode.yaw = cffirmware.modeVelocity
    setpoint.mode.roll = cffirmware.modeDisable
    setpoint.mode.pitch = cffirmware.modeDisable

    setpoint.attitudeRate.yaw = yawDesired
    setpoint.mode.x = cffirmware.modeVelocity
    setpoint.mode.y = cffirmware.modeVelocity
    setpoint.velocity.x = forwardDesired
    setpoint.velocity.y = sidewaysDesired
    #setpoint.attitudeRate.roll = sidewaysDesired/5
    #setpoint.attitudeRate.pitch = forwardDesired/5
    setpoint.attitude.roll = sidewaysDesired
    #setpoint.attitude.pitch = forwardDesired/5
    setpoint.velocity_body = True

    ## Firmware PID bindings
    control = cffirmware.control_t()
    tick = int(robot.getTime()*1000) #this value makes sure that the position controller and attitude controller are always always initiated
    cffirmware.controllerPid(control, setpoint,sensors,state,tick)
    #cffirmware.controllerMellinger(mellinger_control, control, setpoint, sensors, state, tick)


    
    cmd = control.roll/10000
    control.roll = int(cmd)
    cmd = control.pitch/10000
    control.pitch = int(cmd)
    cmd = control.yaw/10000
    control.yaw = int(cmd)


    motors_thrust_uncapped = cffirmware.motors_thrust_uncapped_t()
    motors_thrust_pwm = cffirmware.motors_thrust_pwm_t()
    cffirmware.powerDistribution(control, motors_thrust_uncapped)
    cffirmware.powerDistributionCap(motors_thrust_uncapped, motors_thrust_pwm)
    
    pwm_motors = [motors_thrust_pwm.motors.m1, motors_thrust_pwm.motors.m2, motors_thrust_pwm.motors.m3, motors_thrust_pwm.motors.m4]
    rpm_motors = []
    for pwm in pwm_motors:
        if pwm < 1:
            rpm= 0
        else:
            p = [3.26535711e-01, 3.37495115e+03]
            rpm=np.polyval(p, pwm)
        rpm_motors.append(pwm)

    
    motor_velocities = []
    scaling = 40
    for rpm in rpm_motors:
        motor_velocities.append((rpm/60)/scaling)


    #print(setpoint.attitudeRate.yaw, sensors.gyro.z, control.yaw)

    x.append(robot.getTime())
    y1.append(sensors.gyro.z)
    y2.append(setpoint.attitudeRate.yaw)

    # if x get larger than size_array, remove first element
    if robot.getTime() > max_time:
        x.pop(0)
        y1.pop(0)
        y2.pop(0)

    #plt.cla()
    #plt.plot(x, y1)
    #plt.plot(x, y2)
    #plt.draw()
    #plt.pause(0.001)

    print(motor_velocities)

     ##Todo, remove necessity of this scaling (SI units in firmware)
    m1_motor.setVelocity(-motor_velocities[0])
    m2_motor.setVelocity(motor_velocities[1])
    m3_motor.setVelocity(-motor_velocities[2])
    m4_motor.setVelocity(motor_velocities[3])

    
    ## old way
    '''
    cmd_roll = control.roll /(180.0/np.pi)
    cmd_pitch = control.pitch /(180.0/np.pi)
    cmd_yaw = - control.yaw /(180.0/np.pi)
    cmd_thrust = control.thrust

    ## Motor mixing
    motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
    motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
    motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
    motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw

    scaling = 1500 ##Todo, remove necessity of this scaling (SI units in firmware)
    
    #m1_motor.setVelocity(-motorPower_m1/scaling)
    #m2_motor.setVelocity(motorPower_m2/scaling)
    #m3_motor.setVelocity(-motorPower_m3/scaling)
    #m4_motor.setVelocity(motorPower_m4/scaling)'''
    

    #print(motor_velocities)
    
    past_time = robot.getTime()
    pastXGlobal = xGlobal
    pastYGlobal = yGlobal
    pastZGlobal = zGlobal
    pastRoll = roll
    pastPitch = pitch
    pastYaw = yaw

    pass
