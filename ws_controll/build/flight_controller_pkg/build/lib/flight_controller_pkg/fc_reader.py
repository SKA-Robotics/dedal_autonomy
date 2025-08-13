#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from custom_msgs.msg import DroneStatus, GeoData

# ==============================================================================
# -- MavLink ---------------------------------------------------------------
# ==============================================================================
class MavLinkConfigurator:
    def __init__(self, conn_str='udpin:0.0.0.0:14550'):
        self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat()
        print("✅ Połączono – heartbeat odebrany")

    def end_connection(self):
        self.master.close()

    def calibrate_level(self):
        self.master.mav.command_long_send(
            self.master.target_system,    # ID systemu (autopilota)
            self.master.target_component, # ID komponentu
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, # Komenda kalibracji przed lotem
            0, # Potwierdzenie (0: pierwsze wysłanie)
            0, # 1: gyro calibration, 3: gyro temperature calibration
            0, # 1: magnetometer calibration
            0, # 1: ground pressure calibration
            0, # 1: radio RC calibration, 2: RC trim calibration
            2, # 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            0, # 1: APM: compass/motor interference calibration, 2: airspeed calibration
            0  # 1: ESC calibration, 3: barometer temperature calibration
        )
    
    def EKF_position(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000  # Wysokość względem MSL (Mean Sea Level)
        rel_alt = msg.relative_alt / 1000  # Względem Home
        return [lat, lon, alt, rel_alt]


    def request_message(self, mess_type= 'HIGHRES_IMU'):
        try:
            mes = self.master.recv_match(type= mess_type, blocking= True)
            if mes.msgname == mess_type:
                return mes
            else:
                return self.request_message(mess_type)
        except:
            return None

    def request_message_interval(self, message_id: int, frequency_hz: float):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def tune(self, tune= "t200 o2 a8 a4"):
        target_system = self.master.target_system
        target_component = self.master.target_component

        # Podział melodii na dwie części, jeśli jest dłuższa niż 30 znaków
        tune1 = tune[:30]
        tune2 = tune[30:]

        # Wysłanie wiadomości
        self.master.mav.play_tune_send(
            target_system,
            target_component,
            tune1.encode(),
            tune2.encode()
        )

    def show_me_everything(self):
        while True:
            mes = self.master.recv_msg()
            if mes:
                print(mes)               

# ==============================================================================
# -- DataColection ---------------------------------------------------------------
# ==============================================================================
class MainData:
    def __init__(self):
        self.connection = MavLinkConfigurator()
        battery_freq = 5
        ekf_freq = 10

        battery_status_id = mavlink2.MAVLINK_MSG_ID_BATTERY_STATUS
        ekf_id = mavlink2.MAVLINK_MSG_ID_EKF_STATUS_REPORT
        self.connection.request_message_interval(battery_status_id, battery_freq)
        self.connection.request_message_interval(ekf_id, ekf_freq)

        #self.connection.calibrate_level()
        #self.connection.tune()
    
    def do_magic(self):
        msg = DroneStatus()
        msg.ekf_position = GeoData()
        
        battery_data = self.connection.request_message('BATTERY_STATUS')
        ekf_data = self.connection.EKF_position()

        #print(battery_data.voltages[0]/1000)
        #print(ekf_data)

        msg.is_autonomy_active = False
        msg.battery_voltage = battery_data.voltages[0]/1000
        msg.ekf_position.latitude = ekf_data[0]
        msg.ekf_position.longitude = ekf_data[1]
        msg.ekf_position.altitude = ekf_data[2]
        
        return msg

# ==============================================================================
# -- Node ---------------------------------------------------------------
# ==============================================================================
class imuReaderNode(Node): 
    def __init__(self):
        super().__init__("imu_reader")  
        self.get_logger().info("Node init complete")
        
        self.public_data = MainData()

        self.publisher_ = self.create_publisher(DroneStatus, "drone_status", 100)

        self.timer_ = self.create_timer(1/1, self.timer_function)

    def timer_function(self):
        msg = self.public_data.do_magic()
        self.publisher_.publish(msg)
        #self.get_logger().info("Topic publication")
        
    

# ==============================================================================
# -- Main ---------------------------------------------------------------
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = imuReaderNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
