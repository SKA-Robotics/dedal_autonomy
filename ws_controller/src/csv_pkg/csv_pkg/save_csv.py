#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import ImuData, AccelData, GyroData, EstimatorData, PossitionData
from example_interfaces.msg import String
from datetime import datetime, timedelta
import time
import os
import csv
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration

class LoggingNode(Node):

    def __init__(self):
        super().__init__("UM7_logger")
        
        headers=['timestamp',
            'Accel X', 'Accel Y', 'Accel Z',
            'Gyro X', 'Gyro Y', 'Gyro Z'
            ]
        fc_headers=['timestamp',
            'Accel X', 'Accel Y', 'Accel Z',
            'Gyro X', 'Gyro Y', 'Gyro Z',
            'GPS lat', 'GPS lon', 'GPS alt'
            ]
        
        self.saver_fc = CSVWriter(fc_headers)
        self.saver_fc.change_filename(self.time_now(source="fc"))

        self.saver_um7 = CSVWriter(headers)
        self.saver_um7.change_filename(self.time_now(source="um7"))

        time.sleep(1)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,  # bufor na chwilowe opóźnienia
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            ImuData,
            'fc_imu_data',
            self.fc_listener_callback,
            qos)
        
        self.subscription = self.create_subscription(
            ImuData,
            'um7_imu_data',
            self.um7_listener_callback,
            qos)
        
        # self.create_timer(1.0, self.timer_callback)
        self.subscription
        self.get_logger().info("Init complete")

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

    def time_now(self, source = "-"):
        now = datetime.now()
        filename = "dane-" + source + "-" + now.strftime("%d_%m_%y-%H_%M_%S")
        return filename
    
    def fc_listener_callback(self, msg):
        self.saver_fc.add_row([
            msg.timestamp,
            msg.accel.x, msg.accel.y, msg.accel.z,
            msg.gyro.x, msg.gyro.y, msg.gyro.z,
            msg.latitude, msg.longitude, msg.altitude
            ])
        #self.get_logger().info('Odebrano: "%s"' % msg.timestamp)
        #print(msg)

    def um7_listener_callback(self, msg):
        self.saver_um7.add_row([
            msg.timestamp,
            msg.accel.x, msg.accel.y, msg.accel.z,
            msg.gyro.x, msg.gyro.y, msg.gyro.z
            ])
        
def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    rclpy.spin(node)
    rclpy.shutdown()


class CSVWriter:
    def __init__(self, headers=None, batch_size=5, path='/home/dron/dedal_autonomy/log_data/'):
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