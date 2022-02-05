#!/usr/bin/python

import rospy
from sensor_msgs.msg import BatteryState
import struct
import smbus
import sys
import time
import RPi.GPIO as GPIO



I2C_address=0x36
GPIO_PORT=4

class GeekwormT208():

    def __init__(self, design_capacity=None):

        rospy.loginfo("Setting Up the Geekworm T208 UPS node...")

        # Set up and title the ros node for this code
        rospy.init_node('geekworm_t208')

        # Create publishers for commanding velocity, angle, and robot states
        self._ros_pub_battery_state = rospy.Publisher('/battery_state', BatteryState, queue_size=5)
        self._design_capacity = design_capacity
        self._bus = smbus.SMBus(1) # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_PORT, GPIO.IN)

        rospy.loginfo("Geekworm T208 UPS node publishers corrrectly initialized")

    def readVoltage(self):
        read = self._bus.read_word_data(I2C_address, 2)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        voltage = swapped * 1.25 /1000/16
        return voltage

    def readCapacity(self):
        read = self._bus.read_word_data(I2C_address, 4)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        capacity = swapped/256
        return capacity

    def run(self):
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            bus_voltage = self.readVoltage()
            soc = self.readCapacity()
            if soc > 100:
                soc = 100
            battery_status = BatteryState()
            battery_status.voltage = bus_voltage
            battery_status.cell_voltage = [bus_voltage, bus_voltage, bus_voltage, bus_voltage, bus_voltage, bus_voltage]
            battery_status.percentage = soc
            battery_status.location = "UPS"
            battery_status.present = True
            if soc == 100:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            else:
                if GPIO.input(GPIO_PORT):
                    battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                else:
                    battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self._design_capacity != None:
                battery_status.design_capacity = self._design_capacity
            battery_status.present = True
            battery_status.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            battery_status.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            battery_status.serial_number = 'N/A'
            # print(battery_status)
            self._ros_pub_battery_state.publish(battery_status)
            r.sleep()

if __name__ == "__main__":
    ups = GeekwormT208(design_capacity=rospy.get_param('design_capacity', None))
    ups.run()
