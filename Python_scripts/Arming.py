from pymavlink import mavutil
import time

# Połącz się z autopilotem (zmień np. na '/dev/ttyACM0')
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print("Heartbeat odebrany")


def arm_drone(master: mavutil.mavlink_connection):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,    # param1 = 1 --> arm
        0,    # param2 = 0 --> normal arm (sprawdza safety + prearm)
        0,0,0,0,0
    )
    print("Wysłano polecenie arm, czekam na potwierdzenie...")

    master.motors_armed_wait()
    print("✅ Drone uzbrojony!")


def disarm_drone(master: mavutil.mavlink_connection):
    """
    Rozbraja drona, wysyłając MAV_CMD_COMPONENT_ARM_DISARM z param1=0.
    Czeka na potwierdzenie, że silniki są wyłączone.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,    # param1 = 0 -> disarm
        0,    # param2 = 0 -> standardowe rozbrojenie
        0,0,0,0,0
    )
    print("Wysłano polecenie disarm, oczekuję na potwierdzenie...")
    master.motors_disarmed_wait()
    print("✅ Drone rozbrojony!")

arm_drone(master)