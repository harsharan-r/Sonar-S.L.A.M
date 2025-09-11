#/user/bin/env python3
"""
MPU-6050 I2C Driver for Raspberry Pi

This module handles communication over I2C between a Raspberry Pi
and the MPU-6050 gyroscope/accelerometer sensor. Some code is adapted
from the original Python library by Martijn (2015-2021), with modifications
to fit this project's specific use case.

Original Library:
-----------------
Released under the MIT License
Copyright (c) 2015, 2016, 2017, 2021 Martijn (martijn@mrtijn.nl) and contributors
https://github.com/m-rtijn/mpu6050

Modifications:
--------------
- Added calibration functions to allow for offset correction
- Changed parameters units for ranges to be more readable
"""

import smbus


class MPU:
    """
    Provides a class for the MPU 6050 IMU to get motion and
    oritentation data through the i2c bus
    """
    
    # Gravitational Constant
    GRAVITY_MS2 = 9.80665

    # MPU 6050 Registers
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    ACCEL_XOUT_HIGH = 0x3B
    ACCEL_YOUT_HIGH = 0x3D
    ACCEL_ZOUT_HIGH = 0x3F

    TEMP_OUT_HIGH = 0x41

    GYRO_XOUT_HIGH = 0x43
    GYRO_YOUT_HIGH = 0x45
    GYRO_ZOUT_HIGH = 0x47
    
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    # Pre-defined ranges where keys represent range in units of g
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18 

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Accel and gyro dictionaries
    ACCEL_PARAM = {
            2: {'range': ACCEL_RANGE_2G, 'modifier': ACCEL_SCALE_MODIFIER_2G},
            4: {'range': ACCEL_RANGE_4G, 'modifier': ACCEL_SCALE_MODIFIER_4G},
            8: {'range': ACCEL_RANGE_8G, 'modifier': ACCEL_SCALE_MODIFIER_8G},
            16: {'range': ACCEL_RANGE_16G, 'modifier': ACCEL_SCALE_MODIFIER_16G},
            }

    GYRO_PARAM = {
            250: {'range': GYRO_RANGE_250DEG, 'modifier': GYRO_SCALE_MODIFIER_250DEG},
            500: {'range': GYRO_RANGE_500DEG, 'modifier': GYRO_SCALE_MODIFIER_500DEG},
            1000: {'range': GYRO_RANGE_1000DEG, 'modifier': GYRO_SCALE_MODIFIER_1000DEG},
            2000: {'range': GYRO_RANGE_2000DEG, 'modifier': GYRO_SCALE_MODIFIER_2000DEG},
            }

    # Calibration offsets
    x_accel_offset = 0
    y_accel_offset = 0
    z_accel_offset = 0

    x_gyro_offset = 0
    y_gyro_offset = 0
    z_gyro_offset = 0


    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp    

    def read_i2c_pair(self, register):
        """
        Read two adjacent i2c registers to get a 16 bit signal

        Parameters:
            register (hex): high byte register that should be first in the pair

        Returns:
            hex: combine hex output from the pair of i2c ports
        """

        # Read high and low bytes
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register)

        # Shift high pin 8 bytes left and then add the low bytes
        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def calibrate_accel(self):
        """
        Calculates offsets for accelerometer to zero x and y and 
        set z axis to gravitational constant
        """
        accel_values = self.get_accel_data(g=True)
    
        self.x_accel_offset = -accel_values['x']
        self.y_accel_offset = -accel_values['y']
        self.z_accel_offset = 1-accel_values['z']

    def set_accel_range(self, a_range):
        """
        Sets the sensitivty range of the accelerometer

        Parameters:
            a_range (int): Use a value from the following (in g -- 1 g = 9.81 m/s^2) [2,4,8,16]
        """
        
        # Reset ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the desired range ACCEL_CONFIG register
        try:
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, self.ACCEL_PARAM[a_range]['range'])
        except KeyError as e:
            print("ERROR: range passed is not one of the pre-defined range values: [2,4,8,16]")
        except Exception as e:
            print(f"ERROR: {e}")

        # Confirm correct range is set
        accel_config = self.read_accel_range(raw=True)
        if not self.ACCEL_PARAM[a_range]['range'] == accel_config:     
            print(f"ERROR: config was not assigned correctly, current config: {hex(accel_config)}")
           
        
    def read_accel_range(self, raw=False):
        """
        Reads the accel range sensor is set to 

        Parameter:
            raw (bool, default: False): set true for raw register value and false for the accel in g

        Returns:
            int (raw = True): a value from [2,4,8,16] or -1 if something went wrong
            int (raw = False): returns the int value of raw value of the accel_config register
        """
                
        
        accel_range = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        
        if not raw:
            if accel_range == self.ACCEL_PARAM[2]['range']:
                return 2
            elif accel_range == self.ACCEL_PARAM[4]['range']:
                return 4
            elif accel_range == self.ACCEL_PARAM[8]['range']:
                return 8
            elif accel_range == self.ACCEL_PARAM[16]['range']:
                return 16
            else:
                return -1

        return accel_range
    
    def get_accel_data(self, g=False, calibrate=False):
        """
        Reads and returns x, y and z acceleration values

        Parameters:
            g (bool, default: False) - Set true to get values in units of G and false for M/s^2
            calibrate (bool, default: False) - Set true for calibrated values

        Returns:
            dict: measurement results with keys x, y and z
        """

        accel_range = self.read_accel_range()

        x = self.read_i2c_pair(self.ACCEL_XOUT_HIGH) / self.ACCEL_PARAM[accel_range]['modifier']
        y = self.read_i2c_pair(self.ACCEL_YOUT_HIGH) / self.ACCEL_PARAM[accel_range]['modifier']
        z = self.read_i2c_pair(self.ACCEL_ZOUT_HIGH) / self.ACCEL_PARAM[accel_range]['modifier']

        if calibrate:
            x += self.x_accel_offset
            y += self.y_accel_offset
            z += self.z_accel_offset
        
        accel_values = {'x': x, 'y': y, 'z': z}
        
        if g is False:
            # multiplying all values by gravitational constant
            return {k: v  * self.GRAVITY_MS2 for k, v in accel_values.items()}
        
        return accel_values


    def calibrate_gyro(self):
        """
        Calculates offsets for gyro to zero the values at rest state
        """
        gyro_values = self.get_gyro_data()
    
        self.x_gyro_offset = -gyro_values['x']
        self.y_gyro_offset = -gyro_values['y']
        self.z_gyro_offset = -gyro_values['z']


    def set_gyro_range(self, g_range):
        """
        Sets the sensitivity range of the gyroscope

        Parameters:
            g_range (int): use a value from the following (in deg/s) [250,500,1000,2000]
        """
        # Reset ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the desired range ACCEL_CONFIG register
        try:
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, self.GYRO_PARAM[g_range]['range'])
        except KeyError as e:
            print("ERROR: range passed is not one of the pre-defined range values: [250,500,1000,2000]")
        except Exception as e:
            print(f"ERROR: {e}")

        # Confirm correct range is set
        gyro_config = self.read_gyro_range(raw=True)
        if not self.GYRO_PARAM[g_range]['range'] == gyro_config:         
            print(f"ERROR: config was not assigned correctly, current config: {hex(gyro_config)}")

    def read_gyro_range(self, raw=False):
        """
        Reads the gyro range sensor is set to 

        Parameter:
            raw (bool, default: False): set true for raw register value and false for sensitivity in deg/s

        Returns:
            int (raw = True): a value from [250,500,1000,2000] or -1 if something went wrong
            int (raw = False): returns the int value of raw value of the accel_config register
        """
                
        
        gyro_range = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        
        if not raw:
            if gyro_range == self.GYRO_PARAM[250]['range']:
                return 250
            elif gyro_range == self.GYRO_PARAM[500]['range']:
                return 500
            elif gyro_range == self.GYRO_PARAM[1000]['range']:
                return 1000
            elif gyro_range == self.GYRO_PARAM[2000]['range']:
                return 2000
            else:
                return -1

        return gyro_range

    def get_gyro_data(self, calibrate=False):
        """
        Reads and returns x, y and z gyro values

        Returns:
            dict: measurement results with keys x, y and z
        """

        gyro_range = self.read_gyro_range()

        x = self.read_i2c_pair(self.GYRO_XOUT_HIGH) / self.GYRO_PARAM[gyro_range]['modifier']
        y = self.read_i2c_pair(self.GYRO_YOUT_HIGH) / self.GYRO_PARAM[gyro_range]['modifier']
        z = self.read_i2c_pair(self.GYRO_ZOUT_HIGH) / self.GYRO_PARAM[gyro_range]['modifier']

        if calibrate:
            x += self.x_gyro_offset
            y += self.y_gyro_offset
            z += self.z_gyro_offset

        gyro_values = {'x': x, 'y': y, 'z': z}
                
        return gyro_values

import time

if __name__ == "__main__":
    imu = MPU(0x68)
    
    imu.set_accel_range(16)
    #print(imu.read_accel_range())
    #print(imu.get_accel_data(calibrate=True))
    imu.calibrate_accel()
    #imu.set_accel_range(2)
    #print(imu.read_accel_range())
    #print(imu.get_accel_data(calibrate=True))

    #imu.set_gyro_range(500)
    #print(imu.read_gyro_range())
    #print(imu.get_gyro_data(True))

    imu.calibrate_gyro()
    #imu.set_gyro_range(500)
    #print(imu.read_gyro_range())
    #print(imu.get_gyro_data(True))

    try:
        while(True):
            a_val = imu.get_accel_data(calibrate=True)
            g_val = imu.get_gyro_data(calibrate=True)
            print(f"Accel Values: x: {a_val['x']}, y: {a_val['y']}, z: {a_val['z']}, Gyro Values: x: {g_val['x']}, y: {g_val['y']}, z: {g_val['z']}")    

            time.sleep(0.1)

    finally:
        print("Process has been interuppted")
                
