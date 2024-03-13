#!/usr/bin/python3

# Import necessary libraries
import rospy
import time
import serial
import math
from std_msgs.msg import Header
from vn_driver.msg import Vectornav

# Function to read data from the serial port
def ReadFromSerial(serialPortAddr):
    # Open serial port
    serialPort = serial.Serial(port=serialPortAddr, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
    try:
        # Read data from serial port
        imu_read = serialPort.readline()
        # Decode data from bytes to string
        imu_read = imu_read.decode('utf-8', errors = 'replace').strip()
    except UnicodeDecodeError as e:
        pass
    # Close serial port
    serialPort.close()
    return imu_read

# Function to write to the register of the vectornav
def writeToSerial(serialPortAddr, command):
    serialPort = serial.Serial(port=serialPortAddr, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
    serialPort.write(command.encode('utf-8'))
    # response = serialPort.readline()
    # print("Response:",response.decode('utf-8'))
    serialPort.close()

# Setting the command to write to the register
def set_imu_output_rate(serialPortAddr, rate):
    command = '$VNWRG,07,{}*XX'.format(rate)
    writeToSerial(serialPortAddr, command)

# Function to check if a string contains the "$VNYMR" message identifier
def isVNYMRinString(inputString):
    if "$VNYMR" in inputString:
        print("Great success!")
        return True
    else:
        print("VNYMR not found in string")
        return False

# Function to convert Euler angles to quaternions
def convert_to_quaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Calculate quaternion components
    qw = round(cy * cp * cr + sy * sp * sr, 8)
    qx = round(cy * cp * sr - sy * sp * cr, 8)
    qy = round(sy * cp * sr + cy * sp * cr, 8)
    qz = round(sy * cp * cr - cy * sp * sr, 8)

    quaternion = [qx, qy, qz, qw]
    return quaternion

# Main function
if __name__ == '__main__':
    
    # Initialize ROS node and publisher
    rospy.init_node("imu_publisher", anonymous=True)
    pub = rospy.Publisher("imu", Vectornav, queue_size=10)
    rate = rospy.Rate(40)

    # Get serial port address from ROS parameter server
    serialPortAddr = rospy.get_param("~port")

    # Create custom Vectornav message
    custom_vectornav_msg = Vectornav()
    custom_vectornav_msg.header = Header(frame_id="imu1_frame")
    custom_vectornav_msg.header.seq = 0

    try:
        # Main loop
        while not rospy.is_shutdown():
            # Read data from serial port
            imu_read = ReadFromSerial(serialPortAddr)
            
            #writing to the register to set the output frequency to 40hz
            set_imu_output_rate(serialPortAddr,100)

            # Check if the received data contains the "$VNYMR" message identifier
            if not isVNYMRinString(imu_read):
                continue
            
            # Split the received data into individual components
            imu_read_split = imu_read.split(",")
            
            # Get current time
            currentTime = time.time()
            
            # Set ROS message header timestamp
            custom_vectornav_msg.header.stamp.secs = int(currentTime)
            custom_vectornav_msg.header.stamp.nsecs = int((currentTime - int(currentTime)) * 1e9)

            # Extract Euler angles from the received data
            yaw = float(imu_read_split[1])
            pitch = float(imu_read_split[2])
            roll = float(imu_read_split[3])

            # Convert Euler angles to quaternions
            quaternion = convert_to_quaternion(yaw, pitch, roll)

            # Populate custom Vectornav message with quaternion orientation
            custom_vectornav_msg.imu.header = custom_vectornav_msg.header
            custom_vectornav_msg.imu.orientation.x = quaternion[0]
            custom_vectornav_msg.imu.orientation.y = quaternion[1]
            custom_vectornav_msg.imu.orientation.z = quaternion[2]
            custom_vectornav_msg.imu.orientation.w = quaternion[3]

            # Populate custom Vectornav message with linear acceleration data
            custom_vectornav_msg.imu.linear_acceleration.x = float(imu_read_split[7])
            custom_vectornav_msg.imu.linear_acceleration.y = float(imu_read_split[8])
            custom_vectornav_msg.imu.linear_acceleration.z = float(imu_read_split[9])

            # Populate custom Vectornav message with angular velocity data
            custom_vectornav_msg.imu.angular_velocity.x = float(imu_read_split[10])
            custom_vectornav_msg.imu.angular_velocity.y = float(imu_read_split[11])
            custom_vectornav_msg.imu.angular_velocity.z = float(imu_read_split[12].split("*")[0])

            # Populate custom Vectornav message with magnetic field data (convert from gauss to tesla)
            custom_vectornav_msg.mag_field.header = custom_vectornav_msg.header
            custom_vectornav_msg.mag_field.magnetic_field.x = float(imu_read_split[4]) * 1e-4
            custom_vectornav_msg.mag_field.magnetic_field.y = float(imu_read_split[5]) * 1e-4
            custom_vectornav_msg.mag_field.magnetic_field.z = float(imu_read_split[6]) * 1e-4
            custom_vectornav_msg.imu_read = imu_read
            # Log the custom Vectornav message
            rospy.loginfo(custom_vectornav_msg)
            
            # Publish the custom Vectornav message
            pub.publish(custom_vectornav_msg)
            
            # Sleep to maintain the desired rate
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass


# $VNYMR,-165.970,-037.299,+001.252,+00.2894,+00.0706,+00.7482,-05.961,-00.184,-07.853,+00.000885,-00.000192,-00.000642*69
