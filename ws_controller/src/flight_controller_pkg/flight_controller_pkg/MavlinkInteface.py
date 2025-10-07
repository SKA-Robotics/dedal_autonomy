# MavlinkInterface.py
from pymavlink import mavutil, mavwp


class MavlinkInterface:
    """
    Class for communication with Orange Cube
    """

    # Connection Heartbeat
    def __init__(self, address: str = "udpin:0.0.0.0:14550", logger=None):
        self._log = logger
        self._master = mavutil.mavlink_connection(address, autoreconnect=True)
        self._master.wait_heartbeat()
        self._log.info("✅ MAVLink: Połączono i odebrano heartbeat.")

    def isConnected(self) -> bool:
        if self._master:
            return True
        else:
            raise Error(
                "Mavlink interface did not in fact connect to a flight controller, could not provide master"
            )
            return False

    def send_command_long(
        self,
        system_id,
        component_id,
        command,
        confirmation=0,
        command_param_1=0,
        command_param_2=0,
        command_param_3=0,
        command_param_4=0,
        command_param_5=0,
        command_param_6=0,
        command_param_7=0,
    ):
    """Sends a MAVLink COMMAND_LONG message to a target system and component.

    This method is a wrapper for the pymavlink `command_long_send` function,
    facilitating the transmission of commands that require up to seven
    floating-point parameters.

    Args:
        system_id (int): The system ID of the target vehicle (e.g., 1 for the first drone).
        component_id (int): The component ID of the target (e.g., 1 for autopilot).
        command (int): The MAVLink command ID to be executed. These are defined in
                       the MAVLink specification (e.g., `mavutil.mavlink.MAV_CMD_...`).
        confirmation (int, optional): A confirmation number, typically 0 for the first
                                      transmission. Defaults to 0.
        command_param_1 (float, optional): Parameter 1 for the specific command. Defaults to 0.
        command_param_2 (float, optional): Parameter 2 for the specific command. Defaults to 0.
        command_param_3 (float, optional): Parameter 3 for the specific command. Defaults to 0.
        command_param_4 (float, optional): Parameter 4 for the specific command. Defaults to 0.
        command_param_5 (float, optional): Parameter 5 for the specific command. Defaults to 0.
        command_param_6 (float, optional): Parameter 6 for the specific command. Defaults to 0.
        command_param_7 (float, optional): Parameter 7 for the specific command. Defaults to 0.
    """
    self._master.master.mav.command_long_send(
        system_id,
        component_id,
        command,
        confirmation,
        command_param_1,
        command_param_2,
        command_param_3,
        command_param_4,
        command_param_5,
        command_param_6,
        command_param_7,
    )
    
    def set_position_target_local_ned(
        self,
        type_mask,
        x, y, z,
        vx=0.0, vy=0.0, vz=0.0,
        afx=0.0, afy=0.0, afz=0.0,
        yaw=0.0, yaw_rate=0.0,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED
    ):
        """Sends a SET_POSITION_TARGET_LOCAL_NED MAVLink message.

        This message is used to command the vehicle to a specific position, velocity,
        or acceleration setpoint in the local NED (North-East-Down) frame.
        The `type_mask` parameter is crucial as it specifies which of the other
        parameters should be ignored by the autopilot.

        Args:
            type_mask (int): A bitmask to indicate which fields should be ignored.
                             Combine flags from `mavutil.mavlink` like
                             `POSITION_TARGET_TYPEMASK_..._IGNORE`.
            x (float): Target position on the X-axis (North) in meters.
            y (float): Target position on the Y-axis (East) in meters.
            z (float): Target position on the Z-axis (Down) in meters. Note that
                       a higher value means a lower altitude.
            vx (float, optional): Target velocity on the X-axis in m/s. Defaults to 0.0.
            vy (float, optional): Target velocity on the Y-axis in m/s. Defaults to 0.0.
            vz (float, optional): Target velocity on the Z-axis in m/s. Defaults to 0.0.
            afx (float, optional): Target acceleration on the X-axis in m/s^2. Defaults to 0.0.
            afy (float, optional): Target acceleration on the Y-axis in m/s^2. Defaults to 0.0.
            afz (float, optional): Target acceleration on the Z-axis in m/s^2. Defaults to 0.0.
            yaw (float, optional): Target yaw angle in radians. Defaults to 0.0.
            yaw_rate (float, optional): Target yaw rate in rad/s. Defaults to 0.0.
            coordinate_frame (int, optional): The coordinate frame for the setpoints.
                                             Defaults to MAV_FRAME_LOCAL_NED.
        """
        self._master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (0 to let autopilot set timestamp)
            self._master.target_system,
            self._master.target_component,
            coordinate_frame,
            type_mask,
            x, y, z,
            vx, vy, vz,
            afx, afy, afz,
            yaw,
            yaw_rate
        )

   

    # Idk what is that if don't ask me about it XDDD
    # self._FRAME_BODY_NED = mavutil.mavlink.MAV_FRAME_BODY_NED
    # self._FRAME_LOCAL_NED = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    # self._TYPE_MASK_USE_VELOCITY = 0b0000111111000111   # używaj tylko vx, vy, vz
    # self._TYPE_MASK_USE_POSITION = 0b0000111111111000   # używaj tylko x,y,z (pozycje)
    # self._TYPE_MASK_USE_YAW = 0b000010111111111   # używaj tylko yaw
    # self._TYPE_MASK_USE_ROT_VELOCITY = 0b000000111111111   # używaj tylko vx, vy, vz


#
#
# self._is_armed: bool = False
# self._relative_altitude: float = 0.0
# self._heading: Optional[float] = None
# self._battery_voltage: float = 0.0
# self._heading_deg = None
# self._heading_timestamp = 0.0 #UNIX TIME
# self._telemetry_lock = threading.Lock()
# self._stop_event = threading.Event()
# self._receiver_thread = threading.Thread(target=self._message_receiver_loop, daemon=True)
# self._receiver_thread.start()
#
#
#
# try:
#        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)  # id=33 częśtotliwość 5 Hz
#        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5)              # id=74 częśtotliwość 5 Hz
#        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)             # id=30 częśtotliwość 5 Hz
# except Exception:
#        pass
# self._log.info('MAVLink: Uruchomiono wątek odbiorczy.')
