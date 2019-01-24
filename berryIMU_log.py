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



#Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0



def kalmanFilterY ( accAngle, gyroRate, DT):
	y=0.0
	S=0.0

	global KFangleY
	global Q_angle
	global Q_gyro
	global y_bias
	global YP_00
	global YP_01
	global YP_10
	global YP_11

	KFangleY = KFangleY + DT * (gyroRate - y_bias)

	YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
	YP_01 = YP_01 + ( - DT * YP_11 )
	YP_10 = YP_10 + ( - DT * YP_11 )
	YP_11 = YP_11 + ( + Q_gyro * DT )

	y = accAngle - KFangleY
	S = YP_00 + R_angle
	K_0 = YP_00 / S
	K_1 = YP_10 / S
	
	KFangleY = KFangleY + ( K_0 * y )
	y_bias = y_bias + ( K_1 * y )
	
	YP_00 = YP_00 - ( K_0 * YP_00 )
	YP_01 = YP_01 - ( K_0 * YP_01 )
	YP_10 = YP_10 - ( K_1 * YP_00 )
	YP_11 = YP_11 - ( K_1 * YP_01 )
	
	return KFangleY

def kalmanFilterX ( accAngle, gyroRate, DT):
	x=0.0
	S=0.0

	global KFangleX
	global Q_angle
	global Q_gyro
	global x_bias
	global XP_00
	global XP_01
	global XP_10
	global XP_11


	KFangleX = KFangleX + DT * (gyroRate - x_bias)

	XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
	XP_01 = XP_01 + ( - DT * XP_11 )
	XP_10 = XP_10 + ( - DT * XP_11 )
	XP_11 = XP_11 + ( + Q_gyro * DT )

	x = accAngle - KFangleX
	S = XP_00 + R_angle
	K_0 = XP_00 / S
	K_1 = XP_10 / S
	
	KFangleX = KFangleX + ( K_0 * x )
	x_bias = x_bias + ( K_1 * x )
	
	XP_00 = XP_00 - ( K_0 * XP_00 )
	XP_01 = XP_01 - ( K_0 * XP_01 )
	XP_10 = XP_10 - ( K_1 * XP_00 )
	XP_11 = XP_11 - ( K_1 * XP_01 )
	
	return KFangleX

def main(argv):
      
    #get commandline arguments
    name =''
    duration = 5.0
    printOutput = False
    length = 10
    depth = 3
    prev_rate_gyr = [[0]*depth for _ in range(length)]
    shakeScore = 0
    count=0
    shake_light=0
    flick_light=0
    light_duration=5

    try:
        opts, args = getopt.getopt(argv, "hn:d:p")
    except getopt.GetoptError:
      print ('berryIMU_log.py -n <outfile name> -d <duration> -p')
      sys.exit(2)
    for opt, arg in opts:
      if opt == '-h':
         print ('berryIMU_log.py -n <outfile name> -d <duration> -p')
         sys.exit()
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
    
    
    
        ######################################### 
        #### Median filter for magnetometer ####
        #########################################
        # cycle the table
        for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
            mag_medianTable1X[x] = mag_medianTable1X[x-1]
            mag_medianTable1Y[x] = mag_medianTable1Y[x-1]
            mag_medianTable1Z[x] = mag_medianTable1Z[x-1]
    
        # Insert the latest values    
        mag_medianTable1X[0] = MAGx
        mag_medianTable1Y[0] = MAGy
        mag_medianTable1Z[0] = MAGz    
    
        # Copy the tables
        mag_medianTable2X = mag_medianTable1X[:]
        mag_medianTable2Y = mag_medianTable1Y[:]
        mag_medianTable2Z = mag_medianTable1Z[:]
    
        # Sort table 2
        mag_medianTable2X.sort()
        mag_medianTable2Y.sort()
        mag_medianTable2Z.sort()
    
        # The middle value is the value we are interested in
        MAGx = mag_medianTable2X[MAG_MEDIANTABLESIZE/2];
        MAGy = mag_medianTable2Y[MAG_MEDIANTABLESIZE/2];
        MAGz = mag_medianTable2Z[MAG_MEDIANTABLESIZE/2];
    
    
    
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
    
    
    
        #Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
        CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
    
        #Kalman filter used to combine the accelerometer and gyro values.
        kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
        kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)
    
        if IMU_UPSIDE_DOWN:
            MAGy = -MAGy      #If IMU is upside down, this is needed to get correct heading.
        #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx)/M_PI
    
        #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360
    
    
    
        ####################################################################
        ###################Tilt compensated heading#########################
        ####################################################################
        #Normalize accelerometer raw values.
        if not IMU_UPSIDE_DOWN:        
            #Use these two lines when the IMU is up the right way. Skull logo is facing down
            accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        else:
            #Us these four lines when the IMU is upside down. Skull logo is facing up
            accXnorm = -ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    
        #Calculate pitch and roll
    
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm/math.cos(pitch))
    
    
        #Calculate the new tilt compensated values
        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
     
        #The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
        #is also reversed. This needs to be taken into consideration when performing the calculations
        if(IMU.LSM9DS0):
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS0
        else:
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS1
    
    
    
    
    	#Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
    
        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360
    
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

        # output values
        accl.write(str(rate_gyr_x) + " , " + str(rate_gyr_y)+ " , " + str(rate_gyr_z) + " , " + str(shakeScore) + " , " + str(gyroXangle)  + " , " + str(gyroYangle) + " , " + str(gyroZangle))
        
        accl.write("\n")

        # detect gestures
        if shakeScore > 3000:
            print("shake")
            shake_light = light_duration
            GPIO.output(24,GPIO.HIGH)
        elif abs(rate_gyr_x) > 200:
            print("flick")
            flick_light = light_duration
            GPIO.output(25,GPIO.HIGH)
        else:
            print("static")

        if shake_light > 0:
        	shake_light = shake_light - 1
        else:
        	GPIO.output(24,GPIO.LOW)
        
        if flick_light > 0:
        	flick_light = flick_light - 1
       	else:
        	GPIO.output(25,GPIO.LOW)

        print(str(gyroZangle))

        if printOutput:
            print ("# GRYX %5.2f \tGRYY %5.2f \tGRYZ %5.2f #  " % (rate_gyr_x, rate_gyr_y, rate_gyr_z))

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
        time.sleep(0.1)
        
    print("done recording")
    GPIO.output(23,GPIO.LOW)
    GPIO.output(24,GPIO.LOW)
    GPIO.output(25,GPIO.LOW)

if __name__ == "__main__":
    main(sys.argv[1:])
