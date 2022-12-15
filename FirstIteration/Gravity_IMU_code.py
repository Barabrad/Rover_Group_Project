import smbus
import math
import vect_rot
from time import sleep
#from matplotlib import pyplot as plt

#gathers raw data
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
#angular velocities
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H =  0x47

def round_list(x_list, digits):
    res = x_list.copy()
    for i in range(len(x_list)):
        res[i] = round(x_list[i], digits)
    return res

def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    #bus.write_bye_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = ((high<<8) | low)

    if (value > 32768):
        value -= 65536
    return value

bus = smbus.SMBus(1)
Device_Address = 0x68
MPU_Init()

def getIMUData():
    with open("pvData.csv", 'r') as PVFile:
        oldPos = PVFile.readline().split(',')
        oldVel = PVFile.readline().split(',')
        oldGyro = PVFile.readline().split(',')
    #plt.figure()
    #plt.ion()
    #print(" Accelerometer Data")
    #clears contents
    #Pos_x = []
    #Pos_y = []
    #Pos_z = []
    #p_x_prev = 0
    #p_y_prev = 0
    #p_z_prev = 0
    #p_x = 0
    #p_y = 0
    #p_z = 0
    # TODO: Check out https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    # for dynamically calibrating the IMU using the accelerometer measurements
    #gyro_x = 0
    #gyro_y = 90 # It starts off pitched
    #gyro_z = 0
    # Initial conditions are set by sensorFusion.py, using pvData.csv
    sleep1 = 0.005 #change after testing
    nPerAvg = 1 #10
    g = 9.81 #m/s^2
    gVect = [0, 0, -g]
    # clears contents
    acc_x_i = []
    acc_y_i = []
    acc_z_i = []
    omega_x_i = []
    omega_y_i = []
    omega_z_i = []
    
    # Average out raw IMU angular velocities and translational accelerations to reduce noise
    for i in range(nPerAvg):
        #raw IMU data
        omega_x_i.append(read_raw_data(GYRO_XOUT_H)/131.0) #roll vel
        omega_y_i.append(read_raw_data(GYRO_YOUT_H)/131.0) #pitch vel
        omega_z_i.append(read_raw_data(GYRO_ZOUT_H)/131.0) #yaw vel
        acc_x_i.append(read_raw_data(ACCEL_XOUT_H) * g / 16384.0)
        acc_y_i.append(read_raw_data(ACCEL_YOUT_H) * g / 16384.0)
        acc_z_i.append(read_raw_data(ACCEL_ZOUT_H) * g / 16384.0)
        oFile = open("accData.csv", 'a')
        oFile.write(str(acc_x_i[i]) + "," + str(acc_y_i[i]) + "," + str(acc_z_i[i]) + "\n")
        oFile.close()
        sleep(sleep1)

    dt = sleep1*nPerAvg
    #averaging for noise reduction
    omega_x = (sum(omega_x_i)/nPerAvg)*180/math.pi
    omega_y = (sum(omega_y_i)/nPerAvg)*180/math.pi
    omega_z = (sum(omega_z_i)/nPerAvg)*180/math.pi
    #find angluar positions
    gyro_x = float(oldGyro[0]) + omega_x*dt
    gyro_y = float(oldGyro[1]) + omega_y*dt
    gyro_z = float(oldGyro[2]) + omega_z*dt
    #from vect_rot file
    gVectRot = vect_rot.getRotVect(gVect, gyro_x, gyro_y, gyro_z)

    #print(round(gyro_x, 2), round(gyro_y, 2), round(gyro_z, 2))
    #print(round_list(gVectRot, 2))
    #print(round(sum(acc_x_i)/nPerAvg, 2), round(sum(acc_y_i)/nPerAvg, 2), round(sum(acc_z_i)/nPerAvg, 2))

    #averaging for noise reduction
    acc_x = sum(acc_x_i)/nPerAvg - gVectRot[0]
    acc_y = sum(acc_y_i)/nPerAvg - gVectRot[1]
    acc_z = sum(acc_z_i)/nPerAvg - gVectRot[2]
    #yummy calculus shoutout to SUVAT equations :-)
    #converting to velocities
    vel_x = float(oldVel[0]) + acc_x*dt
    vel_y = float(oldVel[1]) + acc_y*dt
    vel_z = float(oldVel[2]) + acc_z*dt
    #converting to positions
    p_x = float(oldPos[0]) + vel_x*dt #+ .5*acc_x*math.pow(sleep1,2)
    p_y = float(oldPos[1]) + vel_y*dt #+ .5*acc_y*math.pow(sleep1,2)
    p_z = float(oldPos[2]) + vel_z*dt #+ .5*acc_z*math.pow(sleep1,2)

    PVFile = open("pvData.csv", 'w')
    PVFile.write(str(p_x) + "," + str(p_y) + "," + str(p_z) + "\n")
    PVFile.write(str(vel_x) + "," + str(vel_y) + "," + str(vel_z) + "\n")
    PVFile.write(str(gyro_x) + "," + str(gyro_y) + "," + str(gyro_z) + "\n")
    PVFile.close()
    #print(round_list([p_x, p_y, p_z], 2))
    return [p_x, p_y, p_z]
    #print(round_list([acc_x, acc_y, acc_z], 2))
    #plt.plot([p_x_prev, p_x],[p_y_prev, p_y],'-b')

    #p_x_prev = p_x
    #p_y_prev = p_y
    #p_z_prev = p_z

    #plt.close()

#Personal project idea make it store this data and add a plot function
#fix gravity data maybe use quaternion and rpy to take it into account
#also decrease time step to see what happens
