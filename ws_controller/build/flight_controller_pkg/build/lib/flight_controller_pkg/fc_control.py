#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil, mavwp
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from custom_msgs.msg import DroneStatus, GeoData
from example_interfaces.msg import String
from typing import Iterable, List, Sequence, Tuple, Optional
import time
import math
LatLon = Tuple[float, float]
POLYGON = []

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
            0, # 1: radio RC calibration, 2: RC trims calibration
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

    def play_Barka(self):
        tune= "t200 o2 a8 a4"
        #tune = "T140 o3 e2 p8 l4 e d e f e d c c2 p4 d2 e2 f2 f2 p16 f16 f f e d2 d2"
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

    def arm_drone(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,    # param1 = 1 --> arm
            0,    # param2 = 0 --> normal arm (sprawdza safety + prearm)
            0,0,0,0,0
        )
        print("✅ Drone uzbrojony!")

    def disarm_drone(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,    # param1 = 0 -> disarm
            0,    # param2 = 0 -> standardowe rozbrojenie
            0,0,0,0,0
        )
        print("Wysłano polecenie disarm, oczekuję na potwierdzenie...")
        self.master.motors_disarmed_wait()
        print("✅ Drone rozbrojony!")

    def set_landing_mode(self):
        if 'LAND' not in self.master.mode_mapping():
            raise RuntimeError("Tryb LAND nie dostępny")
        mode_id = self.master.mode_mapping()['LAND']
        self.master.mav.set_mode_send(self.master.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id)
        print("➡️ Przełączono do LAND")

    def set_stabilize_mode(self):
        if 'STABILIZE' not in self.master.mode_mapping():
            raise RuntimeError("Tryb STABILIZE nie dostępny")
        mode_id = self.master.mode_mapping()['STABILIZE']
        self.master.mav.set_mode_send(self.master.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id)
        print("➡️ Przełączono do STABILIZE")

    def takeoff_and_hover(self, altitude=5.0):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,0,0,0,0,0, altitude)
        print(f"📍 Start do {altitude} m")
        while True:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
            if not msg:
                print("❗ Brak LOCAL_POSITION_NED")
                break
            z = -msg.z
            print(f"Wysokość: {z:.1f} m")
            if z >= altitude*0.95:
                print("✅ Osiągnięto wysokość – zawis")
                break
            time.sleep(0.5)

    def upload_mission(self, waypoints):
        """
        waypoints: lista krotek (lat, lon, alt)
        Uploaduje misję: najpierw clear, count, potem wysyła MISSION_ITEM
        """
        wp_loader = mavwp.MAVWPLoader()
        seq = 0
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        
        # opcjonalnie dodać punkt TAKEOFF jako pierwszy, tu pomijamy
        for lat, lon, alt in waypoints:
            msg = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.master.target_system, self.master.target_component,
                seq, frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                int(lat*1e7), int(lon*1e7), alt
            )
            wp_loader.add(msg)
            seq += 1
        
        print(f"🔄 Wysyłam {wp_loader.count()} waypointy")
        self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wp_loader.count())

        for _ in range(wp_loader.count()):
            req = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True)
            print(f"Prośba o waypoint seq {req.seq}")
            self.master.mav.send(wp_loader.wp(req.seq))

        ack = self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=10)
        print(f"✅ ACK: {ack.type}" if ack else "❗ Brak ACK")
    
    def offset_to_latlon(self, lat, lon, dx, dy):
        """
        lat, lon – punkt startowy (stopnie)
        dx – przesunięcie wschód (metry)
        dy – przesunięcie północ (metry)
        zwraca: (new_lat, new_lon)
        """
        dlat = dy / 111111.0
        dlon = dx / (111111.0 * math.cos(math.radians(lat)))
        return lat + dlat, lon + dlon

    def read_fence(self, msg):
        POLYGON.append([msg.latitude, msg.longitude])

    def clear_fence(self):
        if len(POLYGON) != 0:
            for i in range(len(POLYGON)):
                POLYGON.pop()

    def set_fence(self):
        sysid, compid = self.master.target_system, self.master.target_component

        # --- BUDOWA ELEMENTÓW FENCE (MISSION_ITEM_INT) ---
        items = []
        vertex_count = len(POLYGON)

        for seq, (lat, lon) in enumerate(POLYGON):
            items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                sysid, compid, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,  # keep-in polygon
                0, 0,
                float(vertex_count), 0, 0, 0,                                  # param1 = liczba wierzchołków
                int(lat * 1e7), int(lon * 1e7), 0.0,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE
            ))

        # --- UPLOAD PRZEZ MISSION PROTOCOL DLA TYPU FENCE ---
        self.master.mav.mission_count_send(sysid, compid, len(items), mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        sent = 0
        while sent < len(items):
            req = self.master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
            if not req:
                raise RuntimeError("Timeout: brak MISSION_REQUEST(INT) podczas uploadu geofence.")
            idx = int(req.seq)
            if 0 <= idx < len(items):
                self.master.mav.send(items[idx])
                sent += 1

        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"MISSION_ACK niepowodzenie (type={getattr(ack,'type',None)})")

        # --- WŁĄCZENIE GEOFENCINGU W ARDUPILOCIE ---
        if vertex_count == 0:
            self.master.mav.param_set_send(sysid, compid, b"FENCE_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
            print("Wyłączono FENCE_ENABLE = 0.")
        elif vertex_count != 0:
            self.master.mav.param_set_send(sysid, compid, b"FENCE_ENABLE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
            print("Wgrano keep-in fence i włączono FENCE_ENABLE = 1.")



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

        self.timer_ = self.create_timer(1/1, self.timer_function)
        
        self.flaskSubscription = self.create_subscription(
            String,
            'flask_commands',
            self.listener_flask_callback,
            10)
        
        self.flaskSubscription = self.create_subscription(
            GeoData,
            'geo_points',
            self.listener_geo_points,
            10)
        
        self.flaskSubscription
        self.publisher_ = self.create_publisher(DroneStatus, "drone_status", 100)


    def timer_function(self):
        msg = self.public_data.do_magic()
        self.publisher_.publish(msg)

    def listener_geo_points(self, msg):
        self.public_data.connection.read_fence(msg)
        print('Fence data received')

    def listener_flask_callback(self, msg):
        #   DISARM
        if(msg.data == 'set_disarm'):
            print('Disarmed')
            self.public_data.connection.disarm_drone()

        #   
        elif(msg.data == 'start_logging'):
            print('Logging started')
            self.public_data.connection.set_fence()

        #   ARM
        elif(msg.data == 'set_arm'):
            print('Arm toggled')
            self.public_data.connection.arm_drone()

        #   LANDING mode
        elif(msg.data == 'land_now'):
            print('Landing')
            self.public_data.connection.set_landing_mode()

        #   STABILIZE mode
        elif(msg.data == 'stabilize'):
            print('Stabilizing')
            self.public_data.connection.set_stabilize_mode()
        
        #   Start mission and hower
        elif(msg.data == 'start_hower'):
            print('Autonomy start')
            self.public_data.connection.takeoff_and_hover()

        #   Cancel mission
        elif(msg.data == 'cancel_mission'):
            print('Mission cancel')
            

        #   Send GeoFence to FC
        elif(msg.data == 'set_geo'):
            print('Fence saved')
            self.public_data.connection.set_fence()

        #   Remove GeoFence points
        elif(msg.data == 'remove_geo'):
            print('Fence removed')
            self.public_data.connection.clear_fence()

        #   Play Barka
        elif(msg.data == 'play_Barka'):
            print('Barking xd')
            self.public_data.connection.play_Barka()


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
