#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rsl_comm_py import UM7Serial
from custom_msgs.msg import ImuData, AccelData, GyroData
import time
from datetime import datetime
import math

#G_force = 9.80665
G_force = 9.81228 # Warsaw g value

# ==============================================================================
# -- DataColection ---------------------------------------------------------------
# ==============================================================================
class MainData:
    def __init__(self):
        self.um7 = UM7Serial(port_name='/dev/ttyUSB0')
        self.um7.port.close()
        self.um7.port.baudrate = 921600
        self.um7.port.open()
        # self.um7.calibrate_accelerometers = 1
        # print("Calibration")
        # time.sleep(1)
        self.set_rates()

        self.msg = ImuData()
        self.msg.accel = AccelData()
        self.msg.gyro = GyroData()
        self.timer_1 = datetime.now()
        self.timer_2 = datetime.now()
    
    def set_rates(self):
        rate = 400
        self.um7.creg_com_rates1 = 10
        self.um7.creg_com_rates2 = 10
        self.um7.creg_com_rates3 = rate
        self.um7.creg_com_rates4 = rate
        self.um7.creg_com_rates5 = 10
        self.um7.creg_com_rates6 = 10
        self.um7.creg_com_rates7 = 10
        print("Rates set")
        time.sleep(1)

    def do_magic(self, publisher):
        for packet in self.um7.recv_all_proc_broadcast(num_packets=400, flush_buffer_on_start=True):
            #print(packet)
            self.msg.timestamp = int(packet.accel_proc_time*1000000)
            self.msg.accel.x = float(packet.accel_proc_x * G_force)
            self.msg.accel.y = float(packet.accel_proc_y * G_force)
            self.msg.accel.z = float(packet.accel_proc_z * G_force)
            self.msg.gyro.x = float(packet.gyro_proc_x*math.pi/180)
            self.msg.gyro.y = float(packet.gyro_proc_y*math.pi/180)
            self.msg.gyro.z = float(packet.gyro_proc_z*math.pi/180)
            publisher.publish(self.msg)
        
            # self.timer_2 = datetime.now()
            # print((self.timer_2.microsecond-self.timer_1.microsecond)/1000)
            # self.timer_1 = self.timer_2
       

        # for packet in self.um7.recv_all_raw_broadcast(1):
        #     msg.timestamp = int(packet.accel_raw_time*1000)
        #     msg.accel.x = float(packet.accel_raw_x/806)
        #     msg.accel.y = float(packet.accel_raw_y/806)
        #     msg.accel.z = float(packet.accel_raw_z/806)
        #     msg.gyro.x = float(packet.gyro_raw_x/806)
        #     msg.gyro.y = float(packet.gyro_raw_y/806)
        #     msg.gyro.z = float(packet.gyro_raw_z/806)
        # print(dir(packet))
        # return msg
        # print(data.time_usec)



class UM7Node(Node):
    def __init__(self):
        super().__init__("imu_reader")
        self.get_logger().info("Node init complete")

        self.public_data = MainData()

        self.publisher_ = self.create_publisher(ImuData, "um7_imu_data", 100)

        self.timer_ = self.create_timer(1, self.timer_function)

    def timer_function(self):
        self.public_data.do_magic(self.publisher_)



def main(args=None):
    rclpy.init(args=args)
    node = UM7Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
