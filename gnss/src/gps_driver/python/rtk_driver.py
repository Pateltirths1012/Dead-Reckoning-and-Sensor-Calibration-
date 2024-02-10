#!/usr/bin/python3

import utm
import rospy
import time
import serial
from std_msgs.msg import Header
from gps_driver.msg import Customrtk
from rospy.exceptions import*

# function to check if the input string contains the GPGGA substring
def isGNGGAinString(inputString):
    if ("$GNGGA" in inputString):
        print("Great success!")
        return True
    else:
        print("GNGGA not found in string")
        return False
        
        
# function to convert latitude and longitude from the format DDmm.mm (degrees, then minutes), to DD.dddd (decimal degrees)
def degMinstoDegDec(LatOrLong):
    dataSplit = LatOrLong.split(".")
    deg = dataSplit[0][:-2]     #removed the last 2 numeric characters
    deg = int(deg)
    min = dataSplit[0][-2:]+ "." +dataSplit[1] #picked only the necessary chars from string
    min = float(min)
    degDec = min / 60.0
    # print (round(deg + degDec, 6))
    return (deg + degDec)

#function to get the coordinates signed
def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if (LatOrLongDir == "W") or (LatOrLongDir == "S"):
        LatOrLong = -1*LatOrLong
    # print(LatOrLong)
    return LatOrLong

#function to convert signed Latitude and longitude to Universal transverse mercator format
def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    # print(UTMVals)
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

#function to convert the gps time to the time needed by ROS
def UTCtoUTCEpoch(UTC):
    hours = int(UTC[:2])
    minutes = int(UTC[2:4])
    seconds = float(UTC[4:])    
    UTCinSecs = hours * 3600 + minutes * 60 + seconds
    TimeSinceEpoch = time.time()
    TimeSinceEpochBOD = TimeSinceEpoch - time.localtime().tm_sec - (time.localtime().tm_min * 60) - (time.localtime().tm_hour * 3600)
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = int((CurrentTime - CurrentTimeSec) * (10**9))
    print(CurrentTime)
    return [CurrentTimeSec, CurrentTimeNsec]

# function to read the data from the gps puck
def ReadFromSerial(serialPortAddr):
    serialPort = serial.Serial(port=serialPortAddr, baudrate=4800, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
    try:    
        gpggaRead = serialPort.readline()
        gpggaRead = gpggaRead.decode('utf-8', errors = 'replace').strip()
        print(gpggaRead)
    except UnicodeDecodeError as e:
        pass
    serialPort.close()
    return gpggaRead


if __name__ == '__main__':

    #Publisher code  
    rospy.init_node("gps_publisher", anonymous=True)
    pub = rospy.Publisher("gps", Customrtk, queue_size=10)
    rate = rospy.Rate(12)

    SerialPortAddr = rospy.get_param("~port")

    # Creating a custom gps message
    custom_rtk_msg = Customrtk()
    custom_rtk_msg.header = Header(frame_id='GPS1_Frame')
    custom_rtk_msg.header.seq = 0

    try:
         
      while not rospy.is_shutdown():
        
            #Read a line from the GPS puck
            gpggaRead = ReadFromSerial(SerialPortAddr)
            # inputString = gpggaRead.decode('latin-1').strip()
            if not isGNGGAinString(gpggaRead):
                continue
            
            gpggaSplit = gpggaRead.split(",") 
            
            UTC = gpggaSplit[1]
            Latitude = gpggaSplit[2]
            LatitudeDir = gpggaSplit[3]
            Longitude = gpggaSplit[4]
            LongitudeDir = gpggaSplit[5]
            altitude = float(gpggaSplit[9])
            HDOP = float(gpggaSplit[8])
            fix_quality = int(gpggaSplit[6])
            Latitude = degMinstoDegDec(Latitude)
            Longitude = degMinstoDegDec(Longitude)
            LatitudeSigned = LatLongSignConvetion(Latitude, LatitudeDir)
            LongitudeSigned = LatLongSignConvetion(Longitude, LongitudeDir)
            UTMVals = convertToUTM(LatitudeSigned, LongitudeSigned)
            CurrentTime = UTCtoUTCEpoch(UTC)
            
            custom_rtk_msg.header.stamp.secs = CurrentTime[0]
            custom_rtk_msg.header.stamp.nsecs = CurrentTime[1]
            custom_rtk_msg.latitude = LatitudeSigned
            custom_rtk_msg.longitude = LongitudeSigned
            custom_rtk_msg.altitude = altitude
            custom_rtk_msg.utm_easting = UTMVals[0]
            custom_rtk_msg.utm_northing = UTMVals[1]
            custom_rtk_msg.zone = UTMVals[2]
            custom_rtk_msg.letter = UTMVals[3]
            custom_rtk_msg.hdop = HDOP
            custom_rtk_msg.fix_quality = fix_quality
            custom_rtk_msg.gpgga_read = gpggaRead
            custom_rtk_msg.header.seq += 1

            rospy.loginfo(custom_rtk_msg)
            pub.publish(custom_rtk_msg)
            rate.sleep()


    except rospy.ROSInterruptException:
           pass
    

    """
    $GNGGA,172705.00,4220.2352990,N,07105.2111316,W,1,02,0.0,15.952,M,-28.727,M,1.0,*52


    $GPGGA,183845.000,4158.4412,N,08754.0202,W,1,05,5.7,100.1,M,-34.1,M,,0000*64
    """
    # python3 serial_emulator.py --file openRTK.txt --device_type gps --loop "yes" --rate 12