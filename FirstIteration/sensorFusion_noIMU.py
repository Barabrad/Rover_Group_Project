import Gravity_IMU_code as imu
import rplidar
import math
import sys
from time import sleep

PORT_NAME = '/dev/ttyUSB0';
lidar = rplidar.RPLidar(PORT_NAME);

# Clear acceleration file
accFile = open("accData.csv", 'w');
accFile.write("a_x,a_y,a_z\n")
accFile.close();

# Clear old position and velocity file
PVFile = open("pvData.csv", 'w');
PVFile.write("0,0,0\n0,0,0\n0,90,0\n"); # The imu is pitched by 90º
PVFile.close();

# Allow user to enter a file name for output
try:
    filename = sys.argv[1];
except:
    filename = "fusedData.csv";
finally:
    oFile = open(filename, 'w');

oFile.write("xL,yL,xI,yI\n");
print("Beginning to write to " + filename);
print("Press Ctrl-C to stop");

i = 1;
try:
    while True:
        if i == 1:
            lidar = rplidar.RPLidar(PORT_NAME);
        sleep(0.1) # Mimic IMU time delay needed for no stop and starting LiDAR
        new_scan, quality, angle, distance = lidar.get_one_packet();
        distance = distance/1000;
        angle = angle + 180; # With the way the LiDAR is set up, 0º is behind the car
        #print("r =", distance, "| angle =", angle);
        xDist = distance*math.cos(angle*math.pi/180);
        # Make LiDAR y negative to account for the IMU setup
        yDist = -distance*math.sin(angle*math.pi/180);
        # z coordinate will be ignored for now
        oFile.write(str(xDist) + "," + str(yDist) + ",0,0\n");
        #print([i, round(xDist,2), round(yDist,2)]);
        print(i);
        i += 1;
        if i == 200:
            lidar.stop();
            i = 1;
except KeyboardInterrupt:
    oFile.close();
    lidar.stop();
    lidar.disconnect();
    print("\nDone");
