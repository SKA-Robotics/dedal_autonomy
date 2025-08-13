from pymavlink import mavutil, mavwp
import time
import math

class Drone:
    def __init__(self, conn_str='udpin:0.0.0.0:14550'):
        self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat()
        print("✅ Połączono – heartbeat odebrany")

    def set_guided_mode(self):
        m = self.master
        if 'GUIDED' not in m.mode_mapping():
            raise RuntimeError("Tryb GUIDED nie dostępny")
        mode_id = m.mode_mapping()['GUIDED']
        m.mav.set_mode_send(m.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id)
        print("➡️ Przełączono do GUIDED")

    def arm(self, force=False):
        param2 = 21196 if force else 0
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, param2, 0,0,0,0,0)
        print(f"Uzbrajam (force={force})…")
        self.master.motors_armed_wait()
        print("✅ Uzbrojono")

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

    def disarm(self, force=False):
        param2 = 21196 if force else 0
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, param2, 0,0,0,0,0)
        print(f"Rozbrajam (force={force})…")
        self.master.motors_disarmed_wait()
        print("🔴 Rozbrojono")

    def upload_mission(self, waypoints):
        """
        waypoints: lista krotek (lat, lon, alt)
        Uploaduje misję: najpierw clear, count, potem wysyła MISSION_ITEM
        """
        m = self.master
        wp_loader = mavwp.MAVWPLoader()
        seq = 0
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        
        # opcjonalnie dodać punkt TAKEOFF jako pierwszy, tu pomijamy
        for lat, lon, alt in waypoints:
            msg = mavutil.mavlink.MAVLink_mission_item_int_message(
                m.target_system, m.target_component,
                seq, frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, 0, 0, 0, 0,
                int(lat*1e7), int(lon*1e7), alt
            )
            wp_loader.add(msg)
            seq += 1
        
        print(f"🔄 Wysyłam {wp_loader.count()} waypointy")
        m.waypoint_clear_all_send()
        m.waypoint_count_send(wp_loader.count())

        for _ in range(wp_loader.count()):
            req = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True)
            print(f"Prośba o waypoint seq {req.seq}")
            m.mav.send(wp_loader.wp(req.seq))

        ack = m.recv_match(type=['MISSION_ACK'], blocking=True, timeout=10)
        print(f"✅ ACK: {ack.type}" if ack else "❗ Brak ACK")

    def start_mission(self):
        # Przełącz w AUTO, co rozpocznie misję
        m = self.master
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mavutil.mavlink.MAV_MODE_AUTO, 0, 0,0,0,0
        )
        print("➡️ Przełączono w tryb AUTO – start misji")

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


def main():
    d = Drone()
    d.set_guided_mode()

    start = (52.2075803742424, 21.0858535766602)
    offsets = [(0,10), (20,0), (0,-15)]
    altitude=5

    mission = []
    cur = start
    for dx, dy in offsets:
        new = d.offset_to_latlon(cur[0], cur[1], dx, dy)
        mission.append((new[0], new[1], altitude))
        cur = new


    d.upload_mission(mission)
    d.arm()
    d.takeoff_and_hover(altitude)
    d.start_mission()

    # opcjonalny monitoring osiągnięcia waypointów...
    time.sleep(30)

    d.disarm()

if __name__ == '__main__':
    main()