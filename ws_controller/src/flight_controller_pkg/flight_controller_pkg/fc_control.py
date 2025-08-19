# =============================================================================
# -- IMPORTY ------------------------------------------------------------------
# =============================================================================
import threading
import math
import time
from typing import Tuple, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import SensorDataQoS

from pymavlink import mavutil, mavwp
from custom_msgs.msg import DroneStatus, GeoData
from example_interfaces.msg import String

LatLon = Tuple[float, float]


# =============================================================================
# -- MAVLink Adapter -----------------------------------------------------------
# =============================================================================
class MavLinkConfigurator:
    def __init__(self, conn_str: str = 'udpin:0.0.0.0:14550', logger=None) -> None:
        self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat()
        self._polygon: List[LatLon] = []
        # logger: callable(level, msg)
        if logger is None:
            self._log = lambda lvl, msg: print(f"[{lvl.upper()}] {msg}")  # fallback
        else:
            self._log = logger
        self._log('info', '✅ Połączono – heartbeat odebrany')

    def end_connection(self) -> None:
        try:
            self.master.close()
        except Exception:
            pass

    # -------------------- Odczyty z timeoutami (bez rekursji) -----------------
    def EKF_position(self, timeout: float = 0.5) -> Optional[list]:
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if not msg:
            return None
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000  # MSL
        rel_alt = msg.relative_alt / 1000  # Względem Home
        return [lat, lon, alt, rel_alt]

    def request_message(self, mess_type: str = 'HIGHRES_IMU', timeout: float = 0.5):
        try:
            mes = self.master.recv_match(type=mess_type, blocking=True, timeout=timeout)
            return mes  # może być None
        except Exception:
            return None

    def request_message_interval(self, message_id: int, frequency_hz: float) -> None:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
        )

    # ---------------------- Przykładowe polecenia trybów ----------------------
    def set_stabilize_mode(self) -> None:
        if 'STABILIZE' not in self.master.mode_mapping():
            raise RuntimeError("Tryb STABILIZE nie dostępny")
        mode_id = self.master.mode_mapping()['STABILIZE']
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self._log('info', '🔁 Ustawiono tryb STABILIZE')

    def set_landing_mode(self) -> None:
        if 'LAND' not in self.master.mode_mapping():
            raise RuntimeError("Tryb LAND nie dostępny")
        mode_id = self.master.mode_mapping()['LAND']
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self._log('info', '🛬 Ustawiono tryb LAND')

    def arm_drone(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0
        )
        self._log('info', '✅ Drone uzbrojony!')

    def disarm_drone(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0
        )
        self._log('info', 'Wysłano polecenie disarm, oczekuję na potwierdzenie...')
        self.master.motors_disarmed_wait()
        self._log('info', '✅ Drone rozbrojony!')

    # ------------------------------- Mission ----------------------------------
    def upload_mission(self, waypoints: list[LatLon]) -> None:
        # Wgrywa prostą misję waypointów (lat, lon, alt).
        wp_loader = mavwp.MAVWPLoader()
        seq = 0
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        for lat, lon, alt in waypoints:
            msg = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.master.target_system, self.master.target_component,
                seq, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0, int(lat*1e7), int(lon*1e7), alt
            )
            wp_loader.add(msg)
            seq += 1

        self._log('info', f'🔄 Wysyłam {wp_loader.count()} waypointy')
        self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wp_loader.count())

        for _ in range(wp_loader.count()):
            req = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=5)
            if not req:
                raise RuntimeError('Timeout przy żądaniu waypointu')
            idx = int(req.seq)
            self.master.mav.send(wp_loader.wp(idx))

        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack:
            raise RuntimeError('Brak MISSION_ACK')
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f'MISSION_ACK niepowodzenie (type={ack.type})')
        self._log('info', '✅ Misja wgrana')

    # ------------------------------ GeoFence ----------------------------------
    def read_fence(self, msg: GeoData) -> None:
        self._polygon.append((msg.latitude, msg.longitude))

    def clear_fence(self) -> None:
        self._polygon.clear()

    def set_fence(self) -> None:
        vertex_count = len(self._polygon)
        if vertex_count == 0:
            # Wyłączenie geofencingu, jeśli nie ma wierzchołków
            self.master.param_set_send(b'FENCE_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            self._log('info', 'FENCE_ENABLE=0 (brak wierzchołków)')
            return

        sysid = self.master.target_system
        compid = self.master.target_component
        items = []

        # Punkt HOME (wymagany przez ArduPilot dla FENCE)
        home = self.master.recv_match(type='HOME_POSITION', blocking=True, timeout=1.0)
        lat_home = getattr(home, 'latitude', 0) / 1e7 if home else 0.0
        lon_home = getattr(home, 'longitude', 0) / 1e7 if home else 0.0
        items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            sysid, compid, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
            0, 0, 0, 0, 0, 0,
            int(lat_home * 1e7), int(lon_home * 1e7), 0.0,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        ))

        # Wierzchołki poligonu (keep-in)
        for seq, (lat, lon) in enumerate(self._polygon, start=1):
            items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
                sysid, compid, seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, 0, float(vertex_count), 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), 0.0,
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE
            ))

        # Upload
        self.master.mav.mission_count_send(sysid, compid, len(items), mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        sent = 0
        while sent < len(items):
            req = self.master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
            if not req:
                raise RuntimeError('Timeout: brak MISSION_REQUEST podczas uploadu geofence.')
            idx = int(req.seq)
            if 0 <= idx < len(items):
                self.master.mav.send(items[idx])
                sent += 1

        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f'MISSION_ACK niepowodzenie (type={getattr(ack, "type", None)})')

        # Włączenie geofencingu
        self.master.param_set_send(b'FENCE_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self._log('info', f'✅ GeoFence ustawiony ({vertex_count} punktów)')


# =============================================================================
# -- Warstwa stanu/telemetrii -------------------------------------------------
# =============================================================================
class MainData:
    # Gromadzi ostatni znany stan; I/O MAVLink w osobnym wątku.
    def __init__(self, logger=None) -> None:
        self._ros_logger = logger
        self.connection = MavLinkConfigurator(logger=self._log)
        self._last_status = DroneStatus(ekf_position=GeoData())
        self._stop = False
        t = threading.Thread(target=self._mav_loop, daemon=True)
        t.start()

    # prosty adapter na logger ROS2
    def _log(self, level: str, msg: str) -> None:
        if self._ros_logger:
            getattr(self._ros_logger, level)(msg)
        else:
            print(msg)

    def _mav_loop(self) -> None:
        while not self._stop:
            try:
                ekf = self.connection.EKF_position(timeout=0.2)
                if ekf:
                    lat, lon, alt, rel_alt = ekf
                    self._last_status.ekf_position.latitude = lat
                    self._last_status.ekf_position.longitude = lon
                    self._last_status.ekf_position.altitude = alt
                bat = self.connection.request_message('BATTERY_STATUS', timeout=0.2)
                if bat and getattr(bat, 'voltages', None):
                    self._last_status.battery_voltage = bat.voltages[0] / 1000.0
            except Exception:
                pass
            time.sleep(0.05)

    # API używane przez node
    def do_magic(self) -> DroneStatus:
        # zwróć kopię prostych pól; przy potrzebie głębokiej kopii – rozważ dataclasses.asdict
        msg = DroneStatus()
        msg.ekf_position = GeoData()
        msg.is_autonomy_active = False
        msg.battery_voltage = getattr(self._last_status, 'battery_voltage', float('nan'))
        msg.ekf_position.latitude = self._last_status.ekf_position.latitude
        msg.ekf_position.longitude = self._last_status.ekf_position.longitude
        msg.ekf_position.altitude = self._last_status.ekf_position.altitude
        return msg

    # GeoFence passthrough
    def read_fence(self, gd: GeoData) -> None:
        self.connection.read_fence(gd)

    def clear_fence(self) -> None:
        self.connection.clear_fence()

    def set_fence(self) -> None:
        self.connection.set_fence()


# =============================================================================
# -- ROS2 Node ----------------------------------------------------------------
# =============================================================================
class FlightControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('FC_controll')
        self.get_logger().info('Node init complete')

        self.public_data = MainData(logger=self.get_logger())

        # Publikator telemetrii – QoS pod dane sensorowe
        self.publisher_ = self.create_publisher(DroneStatus, 'drone_status', SensorDataQoS())

        # Subskrypcje; nie trzeba przechowywać referencji w polach
        self.create_subscription(String, 'flask_commands', self.listener_flask_callback, 10)
        self.create_subscription(GeoData, 'geo_points', self.listener_geo_points, 10)

        # Timer (1 Hz): publikacja ostatniego znanego stanu (bez blokowania)
        self.timer_ = self.create_timer(1.0, self.timer_function)

        # Sprzątanie
        self.add_on_shutdown(self._on_shutdown)

    # ---- Callbacks -----------------------------------------------------------
    def _on_shutdown(self) -> None:
        try:
            self.public_data.connection.end_connection()
        except Exception:
            pass

    def timer_function(self) -> None:
        msg = self.public_data.do_magic()
        self.publisher_.publish(msg)

    def listener_flask_callback(self, msg: String) -> None:
        cmd = msg.data
        self.get_logger().info(f'cmd: {cmd}')

        if cmd == 'arm_now':
            self.public_data.connection.arm_drone()
        elif cmd == 'disarm_now':
            self.public_data.connection.disarm_drone()
        elif cmd == 'land_now':
            self.public_data.connection.set_landing_mode()
        elif cmd == 'stabilize':
            self.public_data.connection.set_stabilize_mode()
        elif cmd == 'start_hover':
            # w oryginale: 'start_hower'
            # TODO: dołącz logikę startu misji/hover jeśli istnieje w systemie
            self.get_logger().info('Autonomy start (hover)')
        elif cmd == 'cancel_mission':
            self.get_logger().info('Mission cancel (not implemented here)')
        elif cmd == 'set_geo':
            self.public_data.set_fence()
        elif cmd == 'remove_geo':
            self.public_data.clear_fence()
        elif cmd == 'barka':
            self.get_logger().info('🎵 Barka (placeholder)')
        else:
            self.get_logger().warn(f'Nieznane polecenie: {cmd}')

    def listener_geo_points(self, msg: GeoData) -> None:
        self.public_data.read_fence(msg)

# =============================================================================
# -- Main ---------------------------------------------------------------------
# =============================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
