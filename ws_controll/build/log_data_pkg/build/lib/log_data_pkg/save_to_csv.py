#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import ImuData, AccelData, GyroData, EstimatorData, PossitionData
from example_interfaces.msg import String
from datetime import datetime, timedelta
import os
import csv


class CSVWriter:
    def __init__(self, filename, headers=None, batch_size=20, path='/home/dron/log_data/'):
        self.path = path
        self.headers = headers
        self.batch_size = batch_size
        self.buffer = []

    def change_filename(self, filename):
        self.filename = self.path + filename
        self._file_initialized = os.path.exists(filename)

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



class SavingNode(Node):
    def __init__(self):
        super().__init__("data_collector")
        self.mode = 'idle'
        self.subscription = self.create_subscription(
            EstimatorData,
            'estimation_data',
            self.listener_callback,
            100)
        self.flaskSubscription = self.create_subscription(
            String,
            'flask_commands',
            self.listener_flask_callback,
            10)
        self.subscription  # zapobiega usunięciu subskrypcji przez garbage collector
        headers=['Timestamp', 
            'Raw data X', 'Raw data Y', 'Raw data Z', 
            'Orientation X', 'Orientation Y', 'Orientation Z',
            'Accel X', 'Accel Y', 'Accel Z',
            'Speed X', 'Speed Y', 'Speed Z',
            'Position X', 'Position Y', 'Position Z'
            ]
        self.writer = CSVWriter(self.time_now(), headers)
        self.get_logger().info("Init complete")

    def listener_callback(self, msg):
        if self.mode == 'logging':
            self.writer.add_row([
                msg.timestamp,
                msg.raw_data.x, msg.raw_data.y, msg.raw_data.z,
                msg.orientation.x, msg.orientation.y, msg.orientation.z,
                msg.accel.x, msg.accel.y, msg.accel.z,
                msg.speed.x, msg.speed.y, msg.speed.z,
                msg.possition.x, msg.possition.y, msg.possition.z
                ])
        #self.get_logger().info('Odebrano: "%s"' % msg.timestamp)
        #print(msg)

    def listener_flask_callback(self, msg):
        if(msg.data == 'start_logging'):
            print('Start data saving')
            self.mode = 'logging'
            self.writer.change_filename(self.time_now())

        elif(msg.data == 'end_logging'):
            print('End data saving')
            self.mode = 'idle'

        elif(msg.data == 'new_logging'):
            print('Start new data file')
            self.mode = 'logging'
            self.writer.change_filename(self.time_now())

    def time_now(self):
        now = datetime.now()
        filename = now.strftime("dane-%d_%m_%y-%H_%M_%S")
        return filename


def main(args=None):
    rclpy.init(args=args)
    node = SavingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()