#!/usr/bin/python
#
#    This program  reads the angles from the acceleromter, gyrscope
#    and mangnetometeron a BerryIMU connected to a Raspberry Pi.
#
#    This program includes two filters (low pass and mdeian) to improve the 
#    values returned from BerryIMU by reducing noise.
#
#
#    http://ozzmaker.com/
#    Both the BerryIMUv1 and BerryIMUv2 are supported
#
#    BerryIMUv1 uses LSM9DS0 IMU
#    BerryIMUv2 uses LSM9DS1 IMU
#




import sys
import getopt
import time
import math
import IMU
import datetime
import os
import RPi.GPIO as GPIO
import socket

# SET UP UDP CONNECTION
# UDP_IP = '131.179.25.231'
# UDP_IP=os.environ["UDP_IP"]
# print os.environ["UDP_IP"]

UDP_PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# If the IMU is upside down (Skull logo facing up), change this value to 1
IMU_UPSIDE_DOWN = 0	


RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  	# [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      	# Complementary filter constant
MAG_LPF_FACTOR = 0.4 	# Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4 	# Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9    	# Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9    	# Median filter table size for magnetometer. Higher = smoother but a longer delay



################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values 
# Calibrating the compass isnt mandatory, however a calibrated 
# compass will result in a more accurate heading value.

magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0


'''
Here is an example:
magXmin =  -1748
magYmin =  -1025
magZmin =  -1876
magXmax =  959
magYmax =  1651
magZmax =  708
Dont use the above values, these are just an example.
'''

def main(argv):
      
    #get commandline arguments
    name =''
    duration = 5.0
    printOutput = False
    length = 50
    depth = 3
    prev_rate_gyr = [[0]*depth for _ in range(length)]
    shakeScore = 0
    count=0
    shake_light=0
    flick_light=0
    light_duration=5
    UDP_IP='131.179.25.231'

    try:
        opts, args = getopt.getopt(argv, "hi:n:d:p")
    except getopt.GetoptError:
      print ('berryIMU_log.py -n <outfile name> -d <duration> -p')
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-h':
         print ('berryIMU_log.py -n <outfile name> -d <duration> -p')
         sys.exit()
      elif opt in ("-i"):
         UDP_IP = arg
      elif opt in ("-n"):
         name = "-"+arg
      elif opt in ("-d"):
         duration = float(arg)
      elif opt in ("-p"):
         printOutput = True
    print ('Name is "', name)
    print ('Duration is %5.2f' % (duration))
    
    #create output files
    

    accl = open("accl"+name+".csv", "w")
	#accl = open("accl"+name+".csv", "w")
    #kalman = open("kalman"+name+".csv", "w")
    #heading_file = open("heading"+name+".csv", "w")
    #complement = open("complement"+name+".csv", "w")
    #gyro = open("gyro"+name+".csv", "w")

    #setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    #assign pins
    GPIO.setup(23,GPIO.OUT)
    GPIO.setup(24,GPIO.OUT)
    GPIO.setup(25,GPIO.OUT)
    
    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0
    CFangleXFiltered = 0.0
    CFangleYFiltered = 0.0
    kalmanX = 0.0
    kalmanY = 0.0
    oldXMagRawValue = 0
    oldYMagRawValue = 0
    oldZMagRawValue = 0
    oldXAccRawValue = 0
    oldYAccRawValue = 0
    oldZAccRawValue = 0
    
    a = datetime.datetime.now()
    
    
    
    #Setup the tables for the mdeian filter. Fill them all with '1' soe we dont get devide by zero error 
    acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
    mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE
    
    IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
    IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
    
    start = datetime.datetime.now()
    cur = datetime.datetime.now()
    
    print("start recording")
    
    GPIO.output(23,GPIO.HIGH)
    GPIO.output(24,GPIO.HIGH)
    GPIO.output(25,GPIO.HIGH)

    time.sleep(.5)

    GPIO.output(24,GPIO.LOW)
    GPIO.output(25,GPIO.LOW)
    
    while (cur - start).total_seconds() < duration:
    
        cur = datetime.datetime.now()
        #Read the accelerometer,gyroscope and magnetometer values
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()
    
    
        #Apply compass calibration    
        MAGx -= (magXmin + magXmax) /2 
        MAGy -= (magYmin + magYmax) /2 
        MAGz -= (magZmin + magZmax) /2 
     
    
        ##Calculate loop Period(LP). How long between Gyro Reads
        b = datetime.datetime.now() - a
        a = datetime.datetime.now()
        LP = b.microseconds/(1000000*1.0)
        #print ("Loop Time | %5.2f|" % ( LP )),
    
    
    
        ############################################### 
        #### Apply low pass filter ####
        ###############################################
        MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR);
        MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR);
        MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR);
        ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR);
        ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR);
        ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR);
    
        oldXMagRawValue = MAGx
        oldYMagRawValue = MAGy
        oldZMagRawValue = MAGz
        oldXAccRawValue = ACCx
        oldYAccRawValue = ACCy
        oldZAccRawValue = ACCz
    
        ######################################### 
        #### Median filter for accelerometer ####
        #########################################
        # cycle the table
        for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
            acc_medianTable1X[x] = acc_medianTable1X[x-1]
            acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
            acc_medianTable1Z[x] = acc_medianTable1Z[x-1]
    
        # Insert the lates values
        acc_medianTable1X[0] = ACCx
        acc_medianTable1Y[0] = ACCy
        acc_medianTable1Z[0] = ACCz    
    
        # Copy the tables
        acc_medianTable2X = acc_medianTable1X[:]
        acc_medianTable2Y = acc_medianTable1Y[:]
        acc_medianTable2Z = acc_medianTable1Z[:]
    
        # Sort table 2
        acc_medianTable2X.sort()
        acc_medianTable2Y.sort()
        acc_medianTable2Z.sort()
    
        # The middle value is the value we are interested in
        ACCx = acc_medianTable2X[ACC_MEDIANTABLESIZE/2];
        ACCy = acc_medianTable2Y[ACC_MEDIANTABLESIZE/2];
        ACCz = acc_medianTable2Z[ACC_MEDIANTABLESIZE/2];
    
        #Convert Gyro raw to degrees per second
        rate_gyr_x =  GYRx * G_GAIN
        rate_gyr_y =  GYRy * G_GAIN
        rate_gyr_z =  GYRz * G_GAIN
    
    
        #Calculate the angles from the gyro. 
        gyroXangle+=rate_gyr_x*LP
        gyroYangle+=rate_gyr_y*LP
        gyroZangle+=rate_gyr_z*LP
    
        #Convert Accelerometer values to degrees
    
        if not IMU_UPSIDE_DOWN:
            # If the IMU is up the correct way (Skull logo facing down), use these calculations
            AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
            AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
        else:
            #Us these four lines when the IMU is upside down. Skull logo is facing up
            AccXangle =  (math.atan2(-ACCy,-ACCz)*RAD_TO_DEG)
            AccYangle =  (math.atan2(-ACCz,-ACCx)+M_PI)*RAD_TO_DEG
    
    
    
        #Change the rotation value of the accelerometer to -/+ 180 and
        #move the Y axis '0' point to up.  This makes it easier to read.
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0
    
        ############################ END ##################################

        #update gyro history
        for x in range(length-1):
            prev_rate_gyr[(length-1)-x][0] = prev_rate_gyr[(length-2)-x][0]
            prev_rate_gyr[(length-1)-x][1] = prev_rate_gyr[(length-2)-x][1]
            prev_rate_gyr[(length-1)-x][2] = prev_rate_gyr[(length-2)-x][2]

        prev_rate_gyr[0][0] = rate_gyr_x
        prev_rate_gyr[0][1] = rate_gyr_y
        prev_rate_gyr[0][2] = rate_gyr_z

        for i in range(length-1):
            shakeScore += abs(prev_rate_gyr[i+1][2] - prev_rate_gyr[i][2])

        # detect angle (abs value because should always opperate in the upper half - paddle is upright)
        angle = str(round(abs(AccXangle)-90))

        #calculate time runnng
        timeElapsed = str((cur - start).total_seconds())

        # Matlab plot can't use string in array of numbers so need numbered status
        outNumber = 0

        # detect gestures
        output =""
        if shakeScore > 6000:
            output = 'shake;' + angle
            outNumber = 2
            shake_light = light_duration
            GPIO.output(24,GPIO.HIGH)
        elif abs(rate_gyr_y) > 800:
            output = 'flick;' + angle + ';' + str(abs(rate_gyr_y))
            outNumber = 1
            flick_light = light_duration
            GPIO.output(25,GPIO.HIGH)
        else:
            output = 'static;' + angle + ";" + timeElapsed + " sec"
            outNumber = 0
        
        # output values
        accl.write(timeElapsed + " , " + str(rate_gyr_x) + " , " + str(rate_gyr_y) + " , " + str(rate_gyr_z) + " , " + str(shakeScore) + " , " + str(gyroXangle)  + " , " + str(gyroYangle) + " , " + str(gyroZangle) + " , " + str(AccXangle) + " , " + str(AccYangle) + " , " + str(outNumber))
        
        accl.write("\n") 

        print(output)
        sock.sendto(output.encode(), (UDP_IP, UDP_PORT))

        if shake_light > 0:
        	shake_light = shake_light - 1
        else:
        	GPIO.output(24,GPIO.LOW)
        
        if flick_light > 0:
        	flick_light = flick_light - 1
       	else:
        	GPIO.output(25,GPIO.LOW)

        if printOutput:
            print (str((cur - start).total_seconds()) + "\n" + "# GRYX %5.2f \tGRYY %5.2f \tGRYZ %5.2f \tAccXangle %5.2f \tAccYangle %5.2f #  " % (rate_gyr_x, rate_gyr_y, rate_gyr_z, AccXangle, AccYangle))

        shakeScore = 0
        #print ("# pGRYX %5.2f \tpGRYY %5.2f \tpGRYZ %5.2f #  " % (prev_rate_gyr[0][0], prev_rate_gyr[0][1], prev_rate_gyr[0][2]))



        if 0:			#Change to '0' to stop showing the magnitude from the accelerometer
            accl.write(str(AccXangle) + " , " + str(AccYangle))
            accl.write("\n")
            #print ("# ACCX Angle %5.2f ACCY Angle %5.2f #  " % (AccXangle, AccYangle)),    
		#
        #if 1:			#Change to '0' to stop  showing the angles from the gyro
        #    gyro.write(str(gyroXangle) + " , " + str(gyroYangle) + " , " + str(gyroYangle))
        #    gyro.write("\n")
        #   #print ("\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)),
		#
        #if 1:			#Change to '0' to stop  showing the angles from the complementary filter
        #    complement.write(str(CFangleX) + " , " + str(CFangleY))
        #    complement.write("\n")
        #    #print ("\t# CFangleX Angle %5.2f   CFangleY Angle %5.2f #" % (CFangleX,CFangleY)),
        #    
        #if 1:			#Change to '0' to stop  showing the heading
        #    heading_file.write(str(heading) + " , " + str(tiltCompensatedHeading))
        #    heading_file.write("\n")
        #    #print ("\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)),
        #    
        #if 1:			#Change to '0' to stop  showing the angles from the Kalman filter
        #    kalman.write(str(kalmanX) + " , " + str(kalmanY))
        #    kalman.write("\n")
            #print ("# kalmanX %5.2f   kalmanY %5.2f #" % (kalmanX,kalmanY)),
		
    
        #print a new line
    
        #slow program down a bit, makes the output more readable
        # originally time.sleep(0.03)
        time.sleep(0.01)
        
    print("done recording")
    GPIO.output(23,GPIO.LOW)
    GPIO.output(24,GPIO.LOW)
    GPIO.output(25,GPIO.LOW)

if __name__ == "__main__":
    main(sys.argv[1:])
