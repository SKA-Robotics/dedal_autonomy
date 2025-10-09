from MavlinkInterface import MavlinkInterface

class MavlinkController:
    def __init__(self,interface : MavlinInterface):
        self.interface = interface
    
    def arm_drone(self):
        self.interface.post_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
    def disarm_drone(self):
        self.interface.post_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)

    
    def reboot_autopilot(self, delay_sec=1):
        """
        Sends a command to reboot the autopilot.

        This function uses the MAV_CMD_PREFL_IGHT_REBOOT_SHUTDOWN command to
        restart the vehicle's main flight controller. A short delay is
        included to allow the command to be processed before the script
        might exit or the connection is lost due to the reboot.

        .. warning:: This is a dangerous operation. Never reboot a vehicle
                     while it is armed or in flight. Use only for pre-flight
                     setup or ground-based recovery procedures.

        Args:
            delay_sec (int, optional): A brief delay in seconds after sending
                                     the command to ensure it is sent before the
                                     connection is lost. Defaults to 1.
        """
        print("Sending reboot command to autopilot...")
        self.interface.post_command_long(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,  # confirmation
            1.0,  # param1: 1 to target autopilot
            1.0,  # param2: 1 to reboot
            0,    # param3: Not used
            0,    # param4: Not used
            0,    # param5: Not used
            0,    # param6: Not used
            0     # param7: Not used
        )
        # Give the command time to be sent before the script might end
        time.sleep(delay_sec)
        print("Reboot command sent. The connection will be lost.")
    def set_position_target_local_ned(self,[x,y,z]):
        
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
    