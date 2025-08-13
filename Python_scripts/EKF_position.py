from pymavlink import mavutil, mavwp

class Drone:
    def __init__(self, conn_str='udpin:0.0.0.0:14550'):
        self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat()
        print("✅ Połączono – heartbeat odebrany")

    def EKF_position(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000  # Wysokość względem MSL (Mean Sea Level)
        rel_alt = msg.relative_alt / 1000  # Względem Home
        print(lat)
        print(lon)
        print(rel_alt)

def main():
    d = Drone()
    d.EKF_position()

if __name__ == '__main__':
    main()