# =============================================================================
# -- IMPORTY ------------------------------------------------------------------
# =============================================================================
import threading
import math
import time
from typing import Tuple, Optional, List
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType

from pymavlink import mavutil, mavwp
from custom_msgs.msg import DroneStatus, GeoData, TagLocation, ImuData, GyroData, AccelData
from example_interfaces.msg import String

LatLon = Tuple[float, float]

# ---- Autonomy flags -------------------------------------------------------
finallPosition = []
finallOrientation = []

@dataclass
class MissionParams:
    xGoal: float = 0.0
    yGoal: float = 0.0
    zGoal: float = 0.0
    yawGoal: float = 0.0
    xVelocity: float = 0.0
    yVelocity: float = 0.0
    zVelocity: float = 0.0
    yawVelocity: float = 0.0
    xReal: float = 0.0
    yReal: float = 0.0
    zReal: float = 0.0
    yawReal: float = 0.0
    duration: float = 10
    elapsed: float = 0
    start: int = 0
    autonomyOn: bool = False
    movementOn: bool = False
    isArmed: bool = False
    heading: float = 0.0
    inSearchMode: bool = False
    searchPoint: int = 0
    durningTakeoff: bool = False
    foundTarget: bool = False
    xTarget: float = 0.0
    yTarget: float = 0.0
    zTarget: float = 0.0
    approachingTarget: bool = False
    targetHoverTimer: float = 0
missionStatus = MissionParams()


# =============================================================================
# -- GeoFence Adapter -----------------------------------------------------------
# =============================================================================
class GeoFenceConfigurator:
    def __init__(self, master, logger=None) -> None:
        self._polygon: List[LatLon] = []
        self._master = master

        if logger is None:
            self._log = lambda lvl, msg: print(f"[{lvl.upper()}] {msg}")  # fallback
        else:
            self._log = logger

    # ------------------------------ GeoFence ----------------------------------
    def read_fence(self, msg: GeoData) -> None:
        self._polygon.append((msg.latitude, msg.longitude))

    def clear_fence(self) -> None:
        self._polygon.clear()

    def set_fence(self) -> None:
        vertex_count = len(self._polygon)

        sysid = self._master.target_system
        compid = self._master.target_component

        if vertex_count == 0:
            # Wyłączenie geofencingu, jeśli nie ma wierzchołków
            #self.master.param_set_send(b'FENCE_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            self._master.mav.param_set_send(sysid, compid, b"FENCE_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
            self._log('info', 'FENCE_ENABLE=0 (brak wierzchołków)')
            return

        items = []

        # Punkt HOME (wymagany przez ArduPilot dla FENCE)
        home = self._master.recv_match(type='HOME_POSITION', blocking=True, timeout=1.0)
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
        self._master.mav.mission_count_send(sysid, compid, len(items), mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        sent = 0
        while sent < len(items):
            req = self._master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5)
            if not req:
                raise RuntimeError('Timeout: brak MISSION_REQUEST podczas uploadu geofence.')
            idx = int(req.seq)
            if 0 <= idx < len(items):
                self._master.mav.send(items[idx])
                sent += 1

        # Włączenie geofencingu
        self._master.param_set_send(b'FENCE_ENABLE', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self._log('info', f'✅ GeoFence ustawiony ({vertex_count} punktów)')

# =============================================================================
# -- Mission controll -------------------------------------------------
# =============================================================================

class MisionController:
    def __init__(self, _master, logger=None) -> None:
        self.master = _master

        if logger is None:
            self._log = lambda lvl, msg: print(f"[{lvl.upper()}] {msg}")  # fallback
        else:
            self._log = logger

        self._FRAME_BODY_NED = mavutil.mavlink.MAV_FRAME_BODY_NED
        self._FRAME_LOCAL_NED = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        self._TYPE_MASK_USE_VELOCITY = 0b0000111111000111   # używaj tylko vx, vy, vz
        self._TYPE_MASK_USE_POSITION = 0b0000111111111000   # używaj tylko x,y,z (pozycje)
        self._TYPE_MASK_USE_YAW = 0b000010111111111   # używaj tylko yaw
        self._TYPE_MASK_USE_ROT_VELOCITY = 0b000000111111111   # używaj tylko vx, vy, vz

    def takeoff_and_hover(self, alt=5.0, loiter_time_s=0):
        # musimy znać aktualną pozycję do LOITER
        pos = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not pos: raise RuntimeError("Brak GLOBAL_POSITION_INT.")
        lat, lon = pos.lat, pos.lon
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

        if loiter_time_s > 0:
            cmd = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
            p1 = loiter_time_s
        else:
            cmd = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
            p1 = 0

        mi0 = mavutil.mavlink.MAVLink_mission_item_int_message(
            self.master.target_system, self.master.target_component, 1, frame,
            cmd, 0, 1,
            p1, 0, 0, float('nan'), lat, lon, alt
        )

        mission = [mi0]

        self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)


        self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, len(mission))
        idx = 0
        while True:
            req = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST'], blocking=True, timeout=5)
            if not req: raise RuntimeError("Timeout przy uploadzie misji.")
            if req.seq == idx:
                self.master.mav.send(mission[idx])
                idx += 1
                if idx == len(mission):
                    break

        # ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=2)
        # if not ack: raise RuntimeError("Brak MISSION_ACK.")
        # start od 0
        self.master.mav.mission_set_current_send(self.master.target_system, self.master.target_component, 0)

    def takeoff(self, alt_m=10):
        # MAV_CMD_NAV_TAKEOFF:
        # param7 = docelowa wysokość (m, względem HOME), param4 = yaw (stopnie), jeśli 0 to bez zmiany
        finallPosition = [0, 0, alt_m]
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,   # param1-4 (opcjonalne)
            0, 0,         # lat, lon (0 = bieżące)
            float(alt_m)  # param7 = wysokość
        )
        self._log('info', f"Komenda TAKEOFF do {alt_m} m wysłana.")

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

        # ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        # if not ack:
        #     raise RuntimeError('Brak MISSION_ACK')
        # if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        #     raise RuntimeError(f'MISSION_ACK niepowodzenie (type={ack.type})')
        self._log('info', '✅ Misja wgrana')



    def _time_boot_ms(self) -> int:
        import time
        if not hasattr(self, "_boot0"):
            self._boot0 = time.monotonic()
        # ms od "startu" obiektu; uint32 wrap dla pewności
        return int((time.monotonic() - self._boot0) * 1000) & 0xFFFFFFFF


    def _send_body_velocity(self, vx: float, vy: float, vz: float) -> None:
        """
        Wyślij pojedynczy setpoint prędkości w BODY_NED:
        X=przód, Y=prawo, Z=dół. Jednostki m/s.
        """
        self.master.mav.set_position_target_local_ned_send(
            self._time_boot_ms(),
            self.master.target_system,
            self.master.target_component,
            self._FRAME_BODY_NED,
            self._TYPE_MASK_USE_VELOCITY,
            0, 0, 0,           # pozycja – ignorowana
            vx, vy, vz,        # prędkości
            0, 0, 0,           # przyspieszenia – ignorowane
            0, 0               # yaw, yaw_rate – ignorowane
        )

    def _send_local_velocity(self, vx: float, vy: float, vz: float) -> None:
        """
        Prędkości w układzie LOCAL_NED (mapa): x=N, y=E, z=Down [m/s].
        """
        self.master.mav.set_position_target_local_ned_send(
            self._time_boot_ms(),
            self.master.target_system,
            self.master.target_component,
            self._FRAME_LOCAL_NED,
            self._TYPE_MASK_USE_VELOCITY,
            0, 0, 0,       # pozycje ignorowane
            vx, vy, vz,    # prędkości [m/s]
            0, 0, 0,       # przyspieszenia ignorowane
            0, 0           # yaw, yaw_rate ignorowane
        )

    def _send_local_velocity_rotation(self, yaw_rate: float) -> None:
        """
        Prędkości w układzie LOCAL_NED (mapa).
        """
        self.master.mav.set_position_target_local_ned_send(
            self._time_boot_ms(),
            self.master.target_system,
            self.master.target_component,
            self._FRAME_LOCAL_NED,
            self._TYPE_MASK_USE_ROT_VELOCITY,
            0, 0, 0,       # pozycje ignorowane
            0, 0, 0,    # prędkości ignorowana
            0, 0, 0,       # przyspieszenia ignorowane
            10,            # yaw ignorowane
            yaw_rate
        )

    def condition_yaw(self, target_angle_deg, yaw_speed_dps, direction, relative=True) -> None:
        """
        direction:  1 = CW, -1 = CCW
        relative:   True = obrót o 'target_angle_deg' względem aktualnego yaw
                    False = obróć do bezwzględnego kursu
        """
        global missionStatus
        if missionStatus.movementOn == False:

            if missionStatus.autonomyOn == False:
                self._log('warn', f'Tryb autonomiczny wyłączony - pomijam')
                return
            
            missionStatus.movementOn = True

            if relative is True:
                missionStatus.yawGoal = target_angle_deg
                missionStatus.duration = target_angle_deg / yaw_speed_dps
            else:
                if missionStatus.heading + 180 < 360:  
                    if missionStatus.yawGoal >= missionStatus.heading and missionStatus.yawGoal <= missionStatus.heading + 180:
                        # obrót CW
                        direction = 1
                        missionStatus.duration = (missionStatus.yawGoal - missionStatus.heading)/yaw_speed_dps
                    else:
                        # obrót CCW
                        direction = -1
                        if missionStatus.yawGoal > missionStatus.heading:
                            missionStatus.duration = (missionStatus.heading + 360 - missionStatus.yawGoal)/yaw_speed_dps
                        else:
                            missionStatus.duration = (missionStatus.heading - missionStatus.yawGoal)/yaw_speed_dps
                else: # Wiemy że missionStatus.heading - 180 < 180
                    if missionStatus.yawGoal >= missionStatus.heading - 180 and missionStatus.yawGoal <= missionStatus.heading:
                        # obrót CCW
                        direction = -1
                        missionStatus.duration = (missionStatus.heading - missionStatus.yawGoal)/yaw_speed_dps
                    else:
                        # obrót CW
                        direction = 1
                        if missionStatus.yawGoal > missionStatus.heading:
                            missionStatus.duration = (missionStatus.yawGoal - missionStatus.heading)/yaw_speed_dps
                        else:
                            missionStatus.duration = (360 - missionStatus.heading - missionStatus.yawGoal)/yaw_speed_dps
                            
                if missionStatus.yawGoal == 0:         
                    missionStatus.yawGoal = target_angle_deg+0.000001

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                float(target_angle_deg),      # param1: docelowy kąt (deg)
                float(yaw_speed_dps),         # param2: prędkość yaw (deg/s)
                float(direction),             # param3: kierunek (1 lub -1)
                1.0 if relative else 0.0,     # param4: relatywny (1) czy absolutny (0)
                0, 0, 0
            )



    def stop(self, repeats: int = 5, rate_hz: int = 10) -> None:
        """
        Wyhamuj do zera (wysyłając kilka ramek 0,0,0).
        """

        period = 1.0 / rate_hz
        for _ in range(repeats):
            self._send_body_velocity(0.0, 0.0, 0.0)
            time.sleep(period)
        self._log('info', '🛑 Zatrzymano setpoint prędkości')

    def move_body_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                           speed_mps: float = 1.0, rate_hz: int = 10) -> None:
        """
        Przemieść się o zadaną odległość (m) w układzie BODY_NED,
        lecąc stałą prędkością (m/s). Utrzymanie wysokości: ustaw dz=0 i vz=0.

        dx>0 = do przodu, dy>0 = w prawo, dz>0 = w dół (uwaga: NED).
        """

        global missionStatus

        if missionStatus.movementOn == False:

            if missionStatus.autonomyOn == False:
                self._log('warn', f'Tryb autonomiczny wyłączony - pomijam')
                return

            missionStatus.movementOn = True

            missionStatus.xGoal = dx
            missionStatus.yGoal = dy
            missionStatus.zGoal = dz

        if speed_mps <= 0:
            raise ValueError("speed_mps musi być > 0")

        # Kierunek jednostkowy w (dx,dy,dz) — aby polecieć dokładnie dystansem L
        L = math.sqrt(dx*dx + dy*dy + dz*dz)
        if L == 0:
            self._log('warning', 'Zadano zerowe przemieszczenie — nic nie robię')
            return

        ux, uy, uz = dx/L, dy/L, dz/L
        vx, vy, vz = ux*speed_mps, uy*speed_mps, uz*speed_mps

        duration = L / speed_mps
        period = 1.0 / rate_hz
        t_end = time.time() + duration

        self._log('info', f'➡️ Ruch BODY_NED: dx={dx:.2f} dy={dy:.2f} dz={dz:.2f} (L={L:.2f} m) '
                          f'v≈({vx:.2f},{vy:.2f},{vz:.2f}) m/s przez ~{duration:.2f} s')


        missionStatus.xVelocity = vx
        missionStatus.yVelocity = vy
        missionStatus.zVelocity = vz
        missionStatus.duration = duration

        # while time.time() < t_end:
        #     self._send_body_velocity(vx, vy, vz)
        #     time.sleep(period)

        # self.stop(rate_hz=rate_hz)
        # self._log('info', '✅ Zrealizowano ruch względny BODY_NED')

    def move_map_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                          speed_mps: float = 1.0, rate_hz: int = 10) -> None:
        """
        Przemieść o (dx,dy,dz) w METRACH względem mapy (LOCAL_NED).
        x>0=północ, y>0=wschód, z>0=w dół. Utrzymanie wysokości -> dz=0 (vz=0).
        Realizacja przez stałą prędkość w LOCAL_NED => tor PROSTY w świecie.
        """
        global missionStatus

        if missionStatus.movementOn == False:

            if missionStatus.autonomyOn == False:
                self._log('warn', f'Tryb autonomiczny wyłączony - pomijam')
                return

            missionStatus.movementOn = True

            missionStatus.xGoal = dx
            missionStatus.yGoal = dy
            missionStatus.zGoal = dz

            if speed_mps <= 0:
                raise ValueError("speed_mps musi być > 0")

            L = math.sqrt(dx*dx + dy*dy + dz*dz)
            if L == 0:
                self._log('warning', 'Zadano zerowe przemieszczenie — pomijam')
                return

            ux, uy, uz = dx/L, dy/L, dz/L
            vx, vy, vz = ux*speed_mps, uy*speed_mps, uz*speed_mps

            duration = L / speed_mps
            period = 1.0 / rate_hz
            t_end = time.time() + duration

            self._log('info', f'➡️ Ruch LOCAL_NED: d=({dx:.2f},{dy:.2f},{dz:.2f}) m, '
                            f'v≈({vx:.2f},{vy:.2f},{vz:.2f}) m/s, t≈{duration:.2f}s')

            missionStatus.xVelocity = vx
            missionStatus.yVelocity = vy
            missionStatus.zVelocity = vz
            missionStatus.duration = duration

            self._log('info', 'Zadano ruch względny LOCAL_NED')

    def rotate_map_relative(self, yaw: float = 0.0, speed_rps: float = 1.0, rate_hz: int = 10) -> None:
        """
        Przemieść o (dx,dy,dz) w METRACH względem mapy (LOCAL_NED).
        x>0=północ, y>0=wschód, z>0=w dół. Utrzymanie wysokości -> dz=0 (vz=0).
        Realizacja przez stałą prędkość w LOCAL_NED => tor PROSTY w świecie.
        """
        global missionStatus
        missionStatus.yawGoal = yaw

        if speed_rps <= 0:
            raise ValueError("speed_rps musi być > 0")

        if yaw == 0:
            self._log('warning', 'Zadano zerowy obrót — pomijam')
            return

        duration = yaw / speed_rps

        self._log('info', f'➡️ Obrót LOCAL_NED: yaw=({yaw:.2f}'
                          f'omega≈({speed_rps:.2f} rps t≈{duration:.2f}s')

        missionStatus.yawVelocity = speed_rps
        missionStatus.duration = duration

        self._log('info', 'Zadano ruch względny LOCAL_NED')

    def move_map_offset_position(self, dx: float, dy: float, dz: float,
                                    hold_s: float = 5.0, rate_hz: int = 10) -> None:
        """
        Przesunięcie pozycyjne: podaj offset (dx,dy,dz) względem aktualnej pozycji LOCAL_NED.
        Wymaga odbierania bieżącej pozycji LOCAL_POSITION_NED.
        """
        import time, math
        # Pobierz aktualną pozycję LOCAL_POSITION_NED
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
        if msg is None:
            self._log('error', 'Brak LOCAL_POSITION_NED – EKF/local frame niegotowy?')
            return
        x0, y0, z0 = msg.x, msg.y, msg.z
        xt, yt, zt = x0 + dx, y0 + dy, z0 + dz

        self._log('info', f'🎯 Cel pozycji LOCAL_NED: ({xt:.2f}, {yt:.2f}, {zt:.2f}) [m]')
        period = 1.0 / rate_hz
        t_end = time.time() + hold_s
        while time.time() < t_end:
            self._send_local_position_target(xt, yt, zt)
            time.sleep(period)

        self._log('info', '✅ Wysłano cel pozycyjny (LOCAL_NED)')

    def move_right(self, distance_m: float = 10.0, speed_mps: float = 1.0, rate_hz: int = 10) -> None:
        """
        Szybka komenda: „w prawo o distance_m”.
        """
        self.move_body_relative(dx=0.0, dy=distance_m, dz=0.0,
                                speed_mps=speed_mps, rate_hz=rate_hz)

    def _read_local_position_ned(self):
        """
        Szybki odczyt ostatniego LOCAL_POSITION_NED z MAVLink (bez blokowania).
        Zwraca tuple: (x, y, z, vx, vy, vz) lub None, jeśli brak świeżych danych.
        """
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg is None:
            return None
        # x,y,z [m], vx,vy,vz [m/s] względem LOCAL_NED
        return (float(getattr(msg, 'x', 0.0)),
                float(getattr(msg, 'y', 0.0)),
                float(getattr(msg, 'z', 0.0)),
                float(getattr(msg, 'vx', 0.0)),
                float(getattr(msg, 'vy', 0.0)),
                float(getattr(msg, 'vz', 0.0)))

    # ======== (Opcjonalnie) Setpoint POZYCJI w LOCAL_NED ========
    # Wysyłanie pozycji w LOCAL_NED też da prostą trajektorię.
    # Tu prosta wersja: wysyłamy cel wielokrotnie przez określony czas.
    def _send_local_position_target(self, x: float, y: float, z: float) -> None:
        """
        Cel pozycyjny w LOCAL_NED (metry od punktu odniesienia EKF).
        """
        self.master.mav.set_position_target_local_ned_send(
            self._time_boot_ms(),
            self.master.target_system,
            self.master.target_component,
            self._FRAME_LOCAL_NED,
            self._TYPE_MASK_USE_POSITION,
            x, y, z,       # pozycja [m]
            0, 0, 0,       # prędkości ignorowane
            0, 0, 0,       # przyspieszenia ignorowane
            0, 0           # yaw, yaw_rate ignorowane
        )
    def clear_mission(self):
        self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        # ACK (opcjonalnie)
        self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)


    def start_mission(self):
        # Wyślij wyraźny start misji
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

# =============================================================================
# -- MAVLink Adapter -----------------------------------------------------------
# =============================================================================
class MavLinkConfigurator:
    def __init__(self, conn_str: str = 'udpin:0.0.0.0:14550', logger=None) -> None:
        self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat()

        if logger is None:
            self._log = lambda lvl, msg: print(f"[{lvl.upper()}] {msg}")
        else:
            self._log = logger
        self._log('info', '✅ Połączono – heartbeat odebrany')

        self.geofence = GeoFenceConfigurator(self.master)
        self.mission = MisionController(self.master)

        # ---- Stan ARM (jak u Ciebie) ----
        self._armed = False
        self._armed_lock = threading.Lock()

        # ---- Stan headingu (nowe) ----
        self._heading_deg = None           # ostatnio znana wartość [0..360)
        self._heading_ts = 0.0             # unix time ostatniej aktualizacji
        self._heading_lock = threading.Lock()

        # Wspólny sygnał stop
        self._stop_evt = threading.Event()

        # Wątek heartbeat (jak u Ciebie)
        self._thr_hb = threading.Thread(target=self._watch_heartbeat, daemon=True)
        self._thr_hb.start()

        # Wątek kompasu/headingu (nowe)
        self._thr_hdg = threading.Thread(target=self._watch_heading, daemon=True)
        self._thr_hdg.start()
        
        # publikacja potrzebnych ramek w zadanej częstotliwości
        try:
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)  # id=33
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)              # id=74
            self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)             # id=30
        except Exception:
            pass

    def end_connection(self) -> None:
        self._stop_evt.set()
        try:
            self._thr_hb.join(timeout=0.5)
            self._thr_hdg.join(timeout=0.5)
        except Exception:
            pass
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

    def _watch_heartbeat(self):
        """Wątek: blokujący odbiór HEARTBEAT i aktualizacja stanu."""
        while not self._stop_evt.is_set():
            # blokująco, ale z timeoutem – wątek da się zatrzymać
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg is None:
                continue  # timeout – spróbuj dalej

            try:
                state = msg.to_dict()
            except Exception:
                continue  # sporadyczne błędy parsowania

            armed_flag = bool(state.get('base_mode', 0) &
                              mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            with self._armed_lock:
                self._armed = armed_flag

    def is_drone_armed(self) -> bool:
        """Szybkie, nieblokujące – zwraca ostatnio znany stan."""
        with self._armed_lock:
            return self._armed      

    def heading_deg(self, timeout: float = 0.5):
        """Zwraca kierunek (kompas) w stopniach [0..360)."""
        # 1) GLOBAL_POSITION_INT.hdg (centy-stopnie)
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is not None and hasattr(msg, 'hdg'):
            # 65535 oznacza "brak danych"
            if msg.hdg is not None and int(msg.hdg) != 65535:
                return (msg.hdg / 100.0) % 360.0

        # 2) VFR_HUD.heading (stopnie)
        msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=timeout)
        if msg is not None and hasattr(msg, 'heading') and msg.heading is not None:
            return float(msg.heading) % 360.0

        # 3) ATTITUDE.yaw (radiany)
        msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
        if msg is not None and hasattr(msg, 'yaw') and msg.yaw is not None:
            return (math.degrees(msg.yaw)) % 360.0

        return None

    def stop(self):
        self._stop_evt.set()
        self._thread.join(timeout=2)

    # -------------------- Nieblokujące API dla kompasu --------------------
    def get_heading_deg(self):
        """Szybkie, nieblokujące – zwraca (heading_deg, age_s) lub (None, None)."""
        with self._heading_lock:
            if self._heading_deg is None:
                return None, None
            return self._heading_deg

    # -------------------- Wątek: heading z kilku źródeł -------------------
    def _watch_heading(self):
        """
        Nasłuchuje tylko ramek z kierunkiem i aktualizuje ostatnio znaną wartość.
        Priorytet:
          1) GLOBAL_POSITION_INT.hdg (centy-stopnie), jeśli != 65535
          2) VFR_HUD.heading (stopnie)
          3) ATTITUDE.yaw (radiany)
        """
        wanted = {'GLOBAL_POSITION_INT', 'VFR_HUD', 'ATTITUDE'}
        while not self._stop_evt.is_set():
            # krótki timeout -> wątek responsywny na _stop_evt
            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue

            mtype = msg.get_type()
            if mtype not in wanted:
                continue

            new_deg = None
            try:
                if mtype == 'GLOBAL_POSITION_INT':
                    # hdg w centy-stopniach; 65535 oznacza brak danych
                    hdg = getattr(msg, 'hdg', None)
                    if hdg is not None and int(hdg) != 65535:
                        new_deg = (float(hdg) / 100.0) % 360.0

                elif mtype == 'VFR_HUD':
                    hdg = getattr(msg, 'heading', None)
                    if hdg is not None:
                        new_deg = (float(hdg)) % 360.0

                elif mtype == 'ATTITUDE':
                    yaw = getattr(msg, 'yaw', None)  # rad
                    if yaw is not None:
                        new_deg = (math.degrees(float(yaw))) % 360.0
            except Exception:
                continue  # sporadyczne błędy parsowania – pomiń

            if new_deg is not None:
                with self._heading_lock:
                    self._heading_deg = new_deg
                    self._heading_ts = time.time()

    # ---------------------- Przykładowe polecenia trybów ----------------------
    def set_mode(self, mode = 'AUTO') -> None:
        if mode not in self.master.mode_mapping():
            raise RuntimeError(f"Tryb {mode} nie dostępny")
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        self._log('info', f'Ustawiono tryb {mode}')

    def arm_drone(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self._log('info', '✅ Drone uzbrojony!')

    def disarm_drone(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        # self._log('info', 'Wysłano polecenie disarm, oczekuję na potwierdzenie...')
        # self.master.motors_disarmed_wait()
        self._log('info', '✅ Drone rozbrojony!')

    # ------------------------------ Inne - dźwięki ----------------------------------

    def tune_short(self, tune= "t200 o2 a8 a4") -> None:
        sysid = self.master.target_system
        compid = self.master.target_component

        self.master.mav.play_tune_send(sysid, compid, tune.encode())
    
    def tune_long(self, tune= "t100 o2 a8 a4 a8 a4") -> None:
        sysid = self.master.target_system
        compid = self.master.target_component

        # Podział melodii na dwie części, jeśli jest dłuższa niż 30 znaków
        tune1 = tune[:30]
        tune2 = tune[30:]

        self.master.mav.play_tune_send(sysid, compid, tune1.encode(), tune2.encode())

    def play_Barka(self) -> None:
        tune = "T140 o3 e2 p8 l4 e d e f e d c c2 p4 d2 e2 f2 f2 p16 f16 f f e d2 d2"
        sysid = self.master.target_system
        compid = self.master.target_component

        tune1 = tune[:30]
        tune2 = tune[30:]

        self.master.mav.play_tune_send(sysid, compid, tune1.encode(), tune2.encode())

# =============================================================================
# -- Warstwa stanu/telemetrii -------------------------------------------------
# =============================================================================
class MainData:
    # Gromadzi ostatni znany stan; I/O MAVLink w osobnym wątku.
    def __init__(self, logger=None) -> None:
        self._ros_logger = logger
        self.connection = MavLinkConfigurator(logger=self._log)
        self._last_status = DroneStatus(ekf_position=GeoData())
        self._last_imu_data = ImuData()
        self._last_imu_data.accel = AccelData()
        self._last_imu_data.gyro = GyroData()
        self._last_imtu_data_timer = 0
        self._stop = False
        t = threading.Thread(target=self._mav_loop, daemon=True)
        t.start()

        imuThread = threading.Thread(target=self._imu_loop, daemon=True)
        imuThread.start()

        # HIGHRES_IMU data request at 250 Hz
        self.connection.request_message_interval(105, 250)

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
                    self._last_status.ekf_position.altitude = rel_alt
                bat = self.connection.request_message('BATTERY_STATUS', timeout=0.2)
                if bat and getattr(bat, 'voltages', None):
                    self._last_status.battery_voltage = bat.voltages[0] / 1000.0
            except Exception:
                pass
            time.sleep(0.001)

    def _imu_loop(self) -> None:
        while not self._stop:            
            try:
                imu_data = self.connection.request_message()
                if imu_data:
                    self._last_imu_data.timestamp = imu_data.time_usec
                    self._last_imu_data.accel.x = imu_data.xacc
                    self._last_imu_data.accel.y = imu_data.yacc
                    self._last_imu_data.accel.z = imu_data.zacc
                    self._last_imu_data.gyro.x = imu_data.xgyro
                    self._last_imu_data.gyro.y = imu_data.ygyro
                    self._last_imu_data.gyro.z = imu_data.zgyro
                    self._last_imu_data.latitude = self._last_status.ekf_position.latitude
                    self._last_imu_data.longitude = self._last_status.ekf_position.longitude
                    self._last_imu_data.altitude = self._last_status.ekf_position.altitude
            except Exception:
                pass

    def publish_imu(self):
        if self._last_imu_data.timestamp > self._last_imtu_data_timer:
            self._last_imtu_data_timer = self._last_imu_data.timestamp
            msg = self._last_imu_data
            return msg

    # API używane przez node
    def do_magic(self) -> DroneStatus:
        # zwróć kopię prostych pól; przy potrzebie głębokiej kopii – rozważ dataclasses.asdict
        global missionStatus
        missionStatus.heading = self.connection.get_heading_deg()
        msg = DroneStatus()
        msg.ekf_position = GeoData()
        msg.is_autonomy_active = missionStatus.autonomyOn
        msg.is_moving = missionStatus.movementOn
        missionStatus.isArmed = self.connection.is_drone_armed()
        msg.is_armed = missionStatus.isArmed
        msg.is_searching = missionStatus.inSearchMode
        msg.is_durning_takeoff = missionStatus.durningTakeoff
        msg.is_target_spotted = missionStatus.foundTarget
        msg.battery_voltage = getattr(self._last_status, 'battery_voltage', float('nan'))
        msg.ekf_position.latitude = self._last_status.ekf_position.latitude
        msg.ekf_position.longitude = self._last_status.ekf_position.longitude
        msg.ekf_position.altitude = self._last_status.ekf_position.altitude
        return msg

    # GeoFence passthrough
    def read_fence(self, gd: GeoData) -> None:
        self.connection.geofence.read_fence(gd)

    def clear_fence(self) -> None:
        self.connection.geofence.clear_fence()

    def set_fence(self) -> None:
        self.connection.geofence.set_fence()

# =============================================================================
# -- ROS2 Node ----------------------------------------------------------------
# =============================================================================
class FlightControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('FC_controll')
        self.get_logger().info('Node init complete')
        
        self.publish_data = MainData(logger=self.get_logger())


        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,  # bufor na chwilowe opóźnienia
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publikator telemetrii – QoS pod dane sensorowe
        self.publisher_ = self.create_publisher(DroneStatus, 'drone_status', qos_profile_sensor_data)
        self.publisher__ = self.create_publisher(ImuData, "fc_imu_data", qos)

        # Subskrypcje; nie trzeba przechowywać referencji w polach
        self.create_subscription(String, 'flask_commands', self.listener_flask_callback, 10)
        self.create_subscription(GeoData, 'geo_points', self.listener_geo_points, 10)
        self.create_subscription(TagLocation, 'goal_location', self.listener_tag_location, 10)

        # Timer
        self.timer_ = self.create_timer(0.25, self.timer_function)
        self.timer__ = self.create_timer(0.1, self.mission_timer)
        self.timer___ = self.create_timer(1/1000, self.imu_publisher_timer)

        # Sprzątanie
        #rclpy.on_shutdown(self._on_shutdown)

    # ---- Callbacks -----------------------------------------------------------
    def _on_shutdown(self) -> None:
        try:
            self.publish_data.connection.end_connection()
        except Exception:
            pass

    def timer_function(self) -> None:
        msg = self.publish_data.do_magic()
        self.publisher_.publish(msg)

    def imu_publisher_timer(self) -> None:
        try:
            msg = self.publish_data.publish_imu()
            self.publisher__.publish(msg)
        except Exception:
            pass

    def mission_timer(self) -> None:
        global missionStatus

        missionPlan = [
                    [0.0, 0.0, 0.0],
                    [0.0, 10.0, 1.0],
                    [1.95, 0.0, 0.0],
                    [-0.4, 1.55, 0.0],
                    [-1.55, 0.4, 0.0],
                    [-1.55, -0.4, 0.0],
                    [-0.4, -1.55, 0.0],
                    [0.4, -1.55, 0.0],
                    [1.55, -0.4, 0.0],
                    [1.55, 0.4, 0.0],
                    [0.4, 1.55, 0.0],
                    [-1.95, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [2.4, 1.6, 0.0],
                    [0.0, -2.3, 0.0],
                    [-1.1, -1.4, 0.0],
                    [0.0, 4.4, 0.0],
                    [-1.3, 0.4, 0.0],
                    [0.0, -5.4, 0.0],
                    [-1.3, 0.4, 0.0],
                    [0.0, 4.4, 0.0],
                    [-1.1, -1.4, 0.0],
                    [0.0, -2.3, 0.0],
                    [2.4, 1.6, 0.0]
                    ]

        if missionStatus.autonomyOn is True:
            if missionStatus.autonomyOn is False:
                missionStatus.inSearchMode = False
                missionStatus.inSearchMode = False
                missionStatus.durningTakeoff = False
                missionStatus.foundTarget = False

            elif missionStatus.movementOn is True:

                if missionStatus.xGoal != 0.0 or missionStatus.yGoal != 0.0 or missionStatus.zGoal != 0.0:
                    if missionStatus.elapsed < 0.95 * missionStatus.duration:
                        if missionStatus.approachingTarget is False:
                            self.publish_data.connection.mission._send_local_velocity(missionStatus.xVelocity, missionStatus.yVelocity, missionStatus.zVelocity)
                        else:
                            self.publish_data.connection.mission._send_body_velocity(missionStatus.xVelocity, missionStatus.yVelocity, missionStatus.zVelocity)
                        missionStatus.elapsed += 0.1

                    # Wyhamuj
                    if missionStatus.elapsed >= 0.95 * missionStatus.duration and missionStatus.elapsed < 1.25 * missionStatus.duration:
                        if missionStatus.approachingTarget is False:
                            self.publish_data.connection.mission._send_local_velocity(0.0, 0.0, 0.0)
                        else:
                            self.publish_data.connection.mission._send_body_velocity(0.0, 0.0, 0.0)
                        missionStatus.elapsed += 0.1

                    if missionStatus.elapsed >= 1.25 * missionStatus.duration:
                        self.get_logger().info('✅ Zrealizowano ruch względny LOCAL_NED')
                        self.publish_data.connection.tune_long()
                        missionStatus.movementOn = False
                        missionStatus.elapsed = 0
                        missionStatus.xGoal = 0
                        missionStatus.yGoal = 0
                        missionStatus.zGoal = 0
                        if missionStatus.approachingTarget is True:
                            missionStatus.xTarget = 0
                            missionStatus.yTarget = 0
                            missionStatus.zTarget = 0
                            self.get_logger().info('Osiągnięto cel?')
                            missionStatus.approachingTarget = False
                            missionStatus.foundTarget = False
                            missionStatus.inSearchMode = True
                            missionStatus.searchPoint = len(missionPlan) + 1


                elif missionStatus.yawGoal != 0.0:
                    if missionStatus.elapsed >= 1.25 * missionStatus.duration:
                        self.get_logger().info('✅ Zrealizowano obrót względny LOCAL_NED')
                        self.publish_data.connection.tune_long()
                        missionStatus.movementOn = False
                        missionStatus.elapsed = 0
                        missionStatus.yawGoal = 0
                    else:
                        missionStatus.elapsed += 0.1

                else:
                    if missionStatus.elapsed >= 1.1 * missionStatus.duration:
                        self.get_logger().info('✅ Zrealizowano procedurę Takeoff')
                        self.publish_data.connection.tune_long()
                        missionStatus.movementOn = False
                        missionStatus.durningTakeoff = False
                        missionStatus.elapsed = 0
                    missionStatus.elapsed += 0.1
            
            elif missionStatus.durningTakeoff is True:
                if missionStatus.isArmed is True:
                    missionStatus.movementOn = True
                    self.get_logger().info('TAKEOFF')
                    missionStatus.duration = 10
                    self.publish_data.connection.mission.takeoff(2)
            
            elif missionStatus.inSearchMode is True and missionStatus.searchPoint == len(missionPlan):
                self.get_logger().info('Koniec poszukiwań - nie udało się znaleźć celu')
                missionStatus.inSearchMode = False
                missionStatus.searchPoint = 0
                self.get_logger().warn('Rozpoczynam procedurę lądowania')
                self.publish_data.connection.set_mode('LAND')            
            
            elif missionStatus.inSearchMode is True:
                fly_speed = 0.25
                
                if missionStatus.searchPoint == 0:
                    self.get_logger().info('Rozpoczynam fazę 1 - start')
                    self.hover_mission()
                    missionStatus.searchPoint += 1

                elif missionStatus.searchPoint == 1:
                    self.get_logger().info('Rozpoczynam fazę 2 - rotacja')
                    self.publish_data.connection.mission.condition_yaw(0, 10, 1, False)
                    missionStatus.searchPoint += 1

                elif missionStatus.searchPoint == len(missionPlan) + 1:
                    self.get_logger().info('Zawis nad celem - oczekuje potwierdzenia')


                elif missionStatus.searchPoint != 0 and missionStatus.searchPoint != len(missionPlan) + 1:
                    dx = missionPlan[missionStatus.searchPoint][0]
                    dy = missionPlan[missionStatus.searchPoint][1]
                    dz = missionPlan[missionStatus.searchPoint][2]
                    self.publish_data.connection.mission.move_map_relative(dx, dy, dz, speed_mps=fly_speed, rate_hz=10)
                    missionStatus.searchPoint += 1

            elif missionStatus.foundTarget == True and missionStatus.approachingTarget is False:
                dx = missionStatus.xTarget
                dy = missionStatus.yTarget
                dz = missionStatus.zTarget
                
                if missionStatus.targetHoverTimer >= 10:
                    self.publish_data.connection.set_mode('LAND')
                    missionStatus.autonomyOn = False
                    missionStatus.targetHoverTimer = 0

                if abs(dx) <= 0.2 and abs(dy) <= 0.2:
                    missionStatus.targetHoverTimer += 0.1
                elif dx != 0 or dy != 0 or dz != 0:
                    missionStatus.approachingTarget = True
                    self.publish_data.connection.mission.move_map_relative(dx, dy, 0.0, speed_mps=0.25, rate_hz=10)

    def hover_mission(self) -> None:
        global missionStatus
        if missionStatus.autonomyOn is True:
            self.publish_data.connection.mission.clear_mission()
            self.get_logger().info('Mission cleared')
            time.sleep(0.5)
            self.publish_data.connection.set_mode('GUIDED')
            self.get_logger().info('GUIDED')
            time.sleep(0.5)

            self.publish_data.connection.arm_drone()
            self.get_logger().info('Uzbrajam drona')

            missionStatus.durningTakeoff = True

        else:
            self.get_logger().warn('Tryb autonomiczny wyłączony - pomijam')

    def listener_flask_callback(self, msg: String) -> None:
        global missionStatus
        cmd = msg.data
        self.get_logger().info(f'cmd: {cmd}')

        if cmd == 'set_arm':
            self.publish_data.connection.arm_drone()
        elif cmd == 'set_disarm':
            self.publish_data.connection.disarm_drone()

        elif cmd == 'land_now':
            self.publish_data.connection.set_mode('LAND')
        elif cmd == 'stabilize':
            self.publish_data.connection.set_mode('STABILIZE')
        elif cmd == 'auto':
            self.publish_data.connection.set_mode('AUTO')
        elif cmd == 'guided':
            self.publish_data.connection.set_mode('GUIDED')

        elif cmd == 'takeoff':
            self.get_logger().info('Takeoff start')
            self.hover_mission()
        elif cmd == 'autonomy_on':
            self.get_logger().info('Autonomy mode on')
            missionStatus.autonomyOn = True
        elif cmd == 'autonomy_off':
            self.get_logger().info('Autonomy mode off')
            missionStatus.autonomyOn = False
        elif cmd == 'mission':
            self.get_logger().info('Mission - Search and Land')
            missionStatus.inSearchMode = True
            self.publish_data.connection.tune_short()
        
        elif cmd == 'test_1':
            self.get_logger().info('Test 1 - Lot 1 m na Północ (0,5 m/s)')
            self.publish_data.connection.tune_short()
            self.publish_data.connection.mission.move_map_relative(dx=1.0, dy=0, dz=0.0, speed_mps=0.5, rate_hz=10)
        elif cmd == 'test_2':
            self.get_logger().info('Test 2 - Lot 1 m na Wschód (0,5 m/s)')
            self.publish_data.connection.tune_short()
            self.publish_data.connection.mission.move_map_relative(dx=0.0, dy=1.0, dz=0.0, speed_mps=0.5, rate_hz=10)
        elif cmd == 'test_3':
            self.get_logger().info('Test 3 - Lot 2 m na Południe i Zachód (0,5 m/s)')
            self.publish_data.connection.tune_short()
            self.publish_data.connection.mission.move_map_relative(dx=-2.0, dy=-2.0, dz=0.0, speed_mps=0.5, rate_hz=10)

        elif cmd == 'test_4':
            self.get_logger().info('Test 4 - Skierowanie się na Północ (5 deg/s)')
            self.publish_data.connection.tune_short()
            self.publish_data.connection.mission.condition_yaw(0, 5, 1, False)
        elif cmd == 'test_5':
            self.get_logger().info('Test 5 - Obrót o 30 deg w prawo (5 deg/s)')
            self.publish_data.connection.tune_short()
            self.publish_data.connection.mission.condition_yaw(30, 5, 1, True)
        elif cmd == 'test_6':
            self.get_logger().info('Brak profilu dla Test 6')
            self.publish_data.connection.mission.condition_yaw(30, 5, -1, True)


        elif cmd == 'set_geo':
            self.publish_data.set_fence()
            self.get_logger().info('Geofence data set')
        elif cmd == 'remove_geo':
            self.publish_data.clear_fence()
            self.get_logger().info('Geofence data cleared')
        
        elif cmd == 'play_Barka':
            self.publish_data.connection.play_Barka()
            self.get_logger().info('🎵 Barka')
        elif cmd == 'inne':
            self.get_logger().info('Its nothing here')

        else:
            self.get_logger().warn(f'Nieznane polecenie: {cmd}')

    def listener_geo_points(self, msg: GeoData) -> None:
        self.get_logger().info('New fence data')
        self.publish_data.read_fence(msg)

    def listener_tag_location(self, msg: TagLocation) -> None:
        global missionStatus
        self.get_logger().info('New goal data')
        if missionStatus.autonomyOn is True:
            if missionStatus.inSearchMode is True:
                self.get_logger().info('Znaleciono cel')
                missionStatus.inSearchMode = False
                missionStatus.foundTarget = True
                missionStatus.approachingTarget = False
                missionStatus.elapsed = missionStatus.duration
                missionStatus.xTarget = msg.x_distance
                missionStatus.yTarget = msg.y_distance
                missionStatus.zTarget = msg.z_distance
            elif missionStatus.movementOn is False:
                dx = msg.x_distance
                dy = msg.y_distance
                dz = msg.z_distance
                self.publish_data.connection.mission.move_map_relative(dx, dy, dz, speed_mps=0.5, rate_hz=10)
        else:
            self.get_logger().warn('Tryb autonomiczny wyłączony - pomijam')


# =============================================================================
# -- Main ---------------------------------------------------------------------
# =============================================================================
def main(args=None) -> None:
    global missionStatus
    rclpy.init(args=args)
    node = FlightControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
