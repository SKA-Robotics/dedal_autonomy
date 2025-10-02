#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import ImuData, AccelData, GyroData, EstimatorData, PossitionData
from example_interfaces.msg import String
from datetime import datetime, timedelta
import time
import os
import csv


class LoggingNode(Node):

    def __init__(self):
        super().__init__("UM7_logger")
        
        headers=['Timestamp',
            'Accel X', 'Accel Y', 'Accel Z',
            'Gyro X', 'Gyro Y', 'Gyro Z'
            ]
        
        self.saver = CSVWriter(self.time_now(), headers)
        self.saver.change_filename(self.time_now())

        time.sleep(1)

        self.subscription = self.create_subscription(
            ImuData,
            'fc_imu_data',
            self.listener_callback,
            100)
        
        # self.create_timer(1.0, self.timer_callback)
        self.subscription
        self.get_logger().info("Init complete")

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

    def time_now(self):
        now = datetime.now()
        filename = now.strftime("dane-fc-%d_%m_%y-%H_%M_%S")
        return filename
    
    def listener_callback(self, msg):
        self.saver.add_row([
            msg.timestamp,
            msg.accel.x, msg.accel.y, msg.accel.z,
            msg.gyro.x, msg.gyro.y, msg.gyro.z
            ])
        #self.get_logger().info('Odebrano: "%s"' % msg.timestamp)
        #print(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    rclpy.spin(node)
    rclpy.shutdown()


class CSVWriter:
    def __init__(self, filename, headers=None, batch_size=20, path='/home/dron/dedal_autonomy/log_data/'):
        self.path = path
        self.headers = headers
        self.batch_size = batch_size
        self.buffer = []
        self._file_initialized = False
        

    def change_filename(self, filename):
        self.filename = self.path + filename + ".csv"
        print(self.filename)
        self._file_initialized = os.path.exists(filename)
        # print(self._file_initialized)

    def add_row(self, row):
        self.buffer.append(row)
        if len(self.buffer) >= self.batch_size:
            self.flush()

    def flush(self):
        mode = 'a' if self._file_initialized else 'w'
        with open(self.filename, mode, newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            if not self._file_initialized and self.headers:
                writer.writerow(self.headers)
                self._file_initialized = True
            writer.writerows(self.buffer)
        self.buffer.clear()

    def flush_remaining(self):
        if self.buffer:
            self.flush()

if __name__ == "__main__":
    main()