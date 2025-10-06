#MavlinkInterface.py

class MavlinkInterface:
    """
    Class for communication with Orange Cube 
    """
    # Connection Heartbeat
    def __init__(self, address: str = 'udpin:0.0.0.0:14550', logger=None):
        self._log = logger
        self._master = mavutil.mavlink_connection(address, autoreconnect=True)
        self._master.wait_heartbeat()
        self._log.info('✅ MAVLink: Połączono i odebrano heartbeat.')
    
    # Idk what is that if don't ask me about it XDDD
    #self._FRAME_BODY_NED = mavutil.mavlink.MAV_FRAME_BODY_NED
    #self._FRAME_LOCAL_NED = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    #self._TYPE_MASK_USE_VELOCITY = 0b0000111111000111   # używaj tylko vx, vy, vz
    #self._TYPE_MASK_USE_POSITION = 0b0000111111111000   # używaj tylko x,y,z (pozycje)
    #self._TYPE_MASK_USE_YAW = 0b000010111111111   # używaj tylko yaw
    #self._TYPE_MASK_USE_ROT_VELOCITY = 0b000000111111111   # używaj tylko vx, vy, vz
#
    #
    #self._is_armed: bool = False
    #self._relative_altitude: float = 0.0
    #self._heading: Optional[float] = None
    #self._battery_voltage: float = 0.0
    #self._heading_deg = None
    #self._heading_timestamp = 0.0 #UNIX TIME  
    #self._telemetry_lock = threading.Lock()
    #self._stop_event = threading.Event()
    #self._receiver_thread = threading.Thread(target=self._message_receiver_loop, daemon=True)
    #self._receiver_thread.start()
#
    #
#
    #try:
    #        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)  # id=33 częśtotliwość 5 Hz
    #        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)              # id=74 częśtotliwość 5 Hz
    #        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)             # id=30 częśtotliwość 5 Hz
    #except Exception:
    #        pass
    #self._log.info('MAVLink: Uruchomiono wątek odbiorczy.')

    def get_master():
        if self._master:
            return self._master
        else:
            raise Error("Mavlink interface did not in fact connect to a flight controller, could not provide master")
            return None
        
    
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
    
    