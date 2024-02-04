#!/usr/bin/python3

import utm
import rospy
import time
import serial
from std_msgs.msg import Header
from gps_driver.msg import Customgps
from rospy.exceptions import*

# function to check if the input string contains the GPGGA substring
def isGPGGAinString(inputString):
    if ("$GPGGA" in inputString):
        print("Great success!")
        return True
    else:
        print("GPGGA not found in string")
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
    return round(deg + degDec, 6)

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
    UTCinSecs = (int(UTC[0:2]) * (3600 + int(UTC[2:4])) + float(UTC[4:]))
    TimeSinceEpoch = time.time()
    TimeSinceEpochBOD = TimeSinceEpoch - time.localtime().tm_sec - (time.localtime().tm_min * 60) - (time.localtime().tm_hour * 3600)
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = int((CurrentTime % 1) * 10**len(str(CurrentTime).split('.')[1]))
    # print(CurrentTime)
    return [CurrentTime, CurrentTimeSec , CurrentTimeNsec]

# function to read the data from the gps puck
def ReadFromSerial(serialPortAddr):
    serialPort = serial.Serial(port=serialPortAddr, baudrate=4800, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
    gpggaRead = serialPort.readline()
    print(gpggaRead)
    serialPort.close()
    return gpggaRead


if __name__ == '__main__':

    #Publisher code  
    rospy.init_node("gps_publisher", anonymous=True)
    pub = rospy.Publisher("custom_gps_topic", Customgps, queue_size=1)
    rate = rospy.Rate(1)

    SerialPortAddr = rospy.get_param("~port")

    # Creating a custom gps message
    custom_gps_msg = Customgps()
    custom_gps_msg.header = Header(frame_id='GPS1_Frame')
    custom_gps_msg.header.seq = 0

    try:
         
      while not rospy.is_shutdown():
        
            #Read a line from the GPS puck
            gpggaRead = ReadFromSerial(SerialPortAddr)
            inputString = gpggaRead.decode('utf-8').strip()
            if not isGPGGAinString(inputString):
                continue
            
            gpggaSplit = inputString.split(",") 
            UTC = gpggaSplit[1]
            Latitude = gpggaSplit[2]
            LatitudeDir = gpggaSplit[3]
            Longitude = gpggaSplit[4]
            LongitudeDir = gpggaSplit[5]
            altitude = gpggaSplit[9]
            HDOP = gpggaSplit[8]
            Latitude = degMinstoDegDec(Latitude)
            Longitude = degMinstoDegDec(Longitude)
            LatitudeSigned = LatLongSignConvetion(Latitude, LatitudeDir)
            LongitudeSigned = LatLongSignConvetion(Longitude, LongitudeDir)
            UTMVals = convertToUTM(LatitudeSigned, LongitudeSigned)
            CurrentTime = UTCtoUTCEpoch(UTC)
            
            custom_gps_msg.header.stamp.secs = CurrentTime[1]
            custom_gps_msg.header.stamp.nsecs = CurrentTime[2]
            custom_gps_msg.latitude = LatitudeSigned
            custom_gps_msg.longitude = LongitudeSigned
            custom_gps_msg.altitude = altitude
            custom_gps_msg.utm_easting = UTMVals[0]
            custom_gps_msg.utm_northing = UTMVals[1]
            custom_gps_msg.zone = UTMVals[2]
            custom_gps_msg.letter = UTMVals[3]
            custom_gps_msg.hdop = HDOP
            custom_gps_msg.gpgga_read = gpggaRead
            custom_gps_msg.header.seq += 1

            rospy.loginfo(custom_gps_msg)
            pub.publish(custom_gps_msg)
            rate.sleep()


    except rospy.ROSInterruptException:
           pass


 
        