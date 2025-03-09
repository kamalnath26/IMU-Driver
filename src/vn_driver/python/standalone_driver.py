#!/usr/bin/env python3
import rospy
import serial
import sys
from vn_driver.msg import Vectornav
import math

class IMU_DRIVER():
    
    def __init__(self, port="/dev/ttyUSB0", baudrate="115200", time_out=1, frequency=40, output_mode=14):
        self.serial_port = port
        self.serial_port_baudrate = baudrate
        self.serial_port_time_out = time_out
        self.serial_port_frequency = frequency
        self.serial_port_output_mode = output_mode
        self.sequence_number = 0
        self.node = rospy.init_node('imu_driver')
        self.publisher = rospy.Publisher('/imu', Vectornav, queue_size=10)
        self.rate = rospy.Rate(40)
        
    def connect(self):
        '''Try to connect to the serial port of IMU.'''
        try:
            rospy.loginfo("Connecting to IMU")
            self.imu_serial = serial.Serial(self.serial_port, self.serial_port_baudrate, timeout=self.serial_port_time_out)
            return True
        except serial.SerialException as error:
            rospy.loginfo("Shutting down...")
            rospy.signal_shutdown("Keyboard Interrupt")
            rospy.logerr_once(f"Unexpected error happened: {error}")
            sys.exit()
    
    def configure_vectornav(self):
        try:
            self.imu_serial.write(b"$VNWRG,07,%d*xx\r" % self.serial_port_frequency)
            self.imu_serial.write(b"$VNWRG,06,%d*xx\r" % self.serial_port_output_mode)
            check = 0
            while check == 0:
                aodf_req = self.imu_serial.write(b"VNRRG,07*xx\r")
                AODF = self.imu_serial.readline()
                if str(self.serial_port_frequency) in str(aodf_req) or str(self.serial_port_frequency) in str(AODF):
                    check = 1

            while check == 1:
                aodf_req = self.imu_serial.write(b"VNRRG,06*xx\r")
                AODF = self.imu_serial.readline()
                if str(self.serial_port_output_mode) in str(aodf_req) or str(self.serial_port_output_mode) in str(AODF):
                    check = 0
            rospy.loginfo('IMU configured')
        except Exception as e:
            rospy.logerr('IMU not configured as per requirement: %s' % str(e))
    
    def get_imu_data(self):
        '''Get data from serial port and return it as a string.'''
        try:
            self.raw_imu_data = self.imu_serial.readline().decode('utf-8').strip()
            self.raw_imu_data_split = self.raw_imu_data.split(",")
            self.exculde_junk_value()
            rospy.loginfo(f"Received raw IMU data: {self.raw_imu_data_split}")
        except Exception as error:
            rospy.logerr(f"Data parsing error: {error}")
    
    def exculde_junk_value(self):
        '''Exclude junk values from the data received from IMU.'''
        exculde_data = self.raw_imu_data_split[-1].split('*')
        self.raw_imu_data_split[-1] = exculde_data[0]
         
    def isVNYMRinString(self, inputString):
        '''Check whether the input string is VNYMR or not
        Args: Input String -> Data must be a string 
        Returns: True if the string is VNYMR, False otherwise'''
        if len(inputString) == 0 or not inputString[0].startswith("$VNYMR"):
            rospy.loginfo(f"NO VNYMR String found in {inputString}")
            return True
        else:
            rospy.loginfo(f"Verified VNYMR String found in {inputString}")
            return False
    
    def update_raw_data(self):
        '''Update raw IMU data from the split string.'''
        if len(self.raw_imu_data_split) < 13:
            rospy.logdebug("Not enough data points to update.")
            return [None] * 12  # Return a list with None values

        try:
            self.raw_yaw = float(self.raw_imu_data_split[1]) if self.raw_imu_data_split[1] else 0.0
            self.raw_pitch = float(self.raw_imu_data_split[2]) if self.raw_imu_data_split[2] else 0.0
            self.raw_roll = float(self.raw_imu_data_split[3]) if self.raw_imu_data_split[3] else 0.0
            
            self.raw_mag_field_x = float(self.raw_imu_data_split[4]) * 1e-4 if self.raw_imu_data_split[4] else 0.0
            self.raw_mag_field_y = float(self.raw_imu_data_split[5]) * 1e-4 if self.raw_imu_data_split[5] else 0.0
            self.raw_mag_field_z = float(self.raw_imu_data_split[6]) * 1e-4 if self.raw_imu_data_split[6] else 0.0
            
            self.raw_acceleration_x = float(self.raw_imu_data_split[7]) if self.raw_imu_data_split[7] else 0.0
            self.raw_acceleration_y = float(self.raw_imu_data_split[8]) if self.raw_imu_data_split[8] else 0.0
            self.raw_acceleration_z = float(self.raw_imu_data_split[9]) if self.raw_imu_data_split[9] else 0.0
            
            self.raw_gyro_x = float(self.raw_imu_data_split[10]) if self.raw_imu_data_split[10] else 0.0
            self.raw_gyro_y = float(self.raw_imu_data_split[11]) if self.raw_imu_data_split[11] else 0.0
            self.raw_gyro_z = float(self.raw_imu_data_split[12]) if self.raw_imu_data_split[12] else 0.0

        except ValueError as e:
            rospy.logerr(f"ValueError: {e}")
            return [0.0] * 12  # Return a list with default values

        return [self.raw_yaw, self.raw_pitch, self.raw_roll, self.raw_mag_field_x, self.raw_mag_field_y, 
                self.raw_mag_field_z, self.raw_acceleration_x, self.raw_acceleration_y, 
                self.raw_acceleration_z, self.raw_gyro_x, self.raw_gyro_y, self.raw_gyro_z]
    
    def euler_to_quaternion(self, yaw, pitch, roll):
        """Convert Euler angles (yaw, pitch, roll) to a quaternion."""
        
        # Convert degrees to radians if necessary
        yaw = yaw * math.pi / 180.0
        pitch = pitch * math.pi / 180.0
        roll = roll * math.pi / 180.0
        
        # Calculate the quaternion components
        self.q_w = (math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) +
            math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
        
        self.q_x = (math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) -
            math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
        
        self.q_y = (math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) +
            math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2))
        
        self.q_z = (math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) -
            math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2))
        
        return self.q_x, self.q_y, self.q_z, self.q_w

    def update_ros_msg_and_publish(self):
        '''Update and publish the ROS message.'''
        self.msg = Vectornav()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.seq = self.sequence_number 
        self.msg.header.frame_id = 'imu1_frame'
        self.msg.imu.header.seq = self.sequence_number
        self.msg.imu.header.frame_id = 'imu1_frame'
        self.msg.imu.header.stamp = rospy.Time.now()
        self.msg.imu.orientation.x = self.q_x
        self.msg.imu.orientation.y = self.q_y
        self.msg.imu.orientation.z = self.q_z
        self.msg.imu.orientation.w = self.q_w
        self.msg.mag_field.header.seq = self.sequence_number
        self.msg.mag_field.header.frame_id = 'imu1_frame'
        self.msg.mag_field.header.stamp = rospy.Time.now()
        self.msg.mag_field.magnetic_field.x = self.raw_mag_field_x
        self.msg.mag_field.magnetic_field.y = self.raw_mag_field_y
        self.msg.mag_field.magnetic_field.z = self.raw_mag_field_z
        self.msg.imu.linear_acceleration.x = self.raw_acceleration_x
        self.msg.imu.linear_acceleration.y = self.raw_acceleration_y
        self.msg.imu.linear_acceleration.z = self.raw_acceleration_z
        self.msg.imu.angular_velocity.x = self.raw_gyro_x
        self.msg.imu.angular_velocity.y = self.raw_gyro_y
        self.msg.imu.angular_velocity.z = self.raw_gyro_z
        self.msg.vn_read = self.raw_imu_data
        self.sequence_number += 1
                    
        rospy.loginfo(self.msg)
        self.publisher.publish(self.msg)
        self.rate.sleep()
          
    def start_driver(self):
        '''Main loop for reading IMU data and publishing it.'''
        while not rospy.is_shutdown():
            self.get_imu_data()
            if self.isVNYMRinString(self.raw_imu_data_split):
                continue
            if len(self.raw_imu_data_split) < 13:
                rospy.loginfo("Missing a few data points, skipping this iteration")
                continue
            self.updated_values = self.update_raw_data()
            if None in self.updated_values or 0.0 in self.updated_values:
                rospy.loginfo("Missing a few data points, skipping this iteration")
                continue
            self.euler_to_quaternion(self.raw_yaw, self.raw_pitch, self.raw_roll)
            self.update_ros_msg_and_publish()


if __name__ == "__main__":
    try:
        rospy.init_node('imu_driver')
        if len(sys.argv) == 2:
            port_number = sys.argv[1]
        else:
            port_number = rospy.get_param('~port', '/dev/ttyUSB0')
        imu_baudrate = rospy.get_param('~baudrate', '115200')
        connection_timeout = rospy.get_param('~connection_timeout', 5)
        imu_frequency = rospy.get_param('~frequency', 40)
        imu_data_output_mode = rospy.get_param('~output_mode', 14)
        rospy.loginfo(f"Starting the IMU Driver with port: {port_number}, baudrate: {imu_baudrate}, connection timeout: {connection_timeout}, imu frequency: {imu_frequency} and imu data output mode: {imu_data_output_mode}")
        imu = IMU_DRIVER(port=port_number, baudrate=imu_baudrate, time_out=connection_timeout, frequency=imu_frequency, output_mode=imu_data_output_mode)
        imu.connect()
        imu.configure_vectornav()
        imu.start_driver()
    except Exception as error:
        rospy.loginfo(f"Error: {error}")