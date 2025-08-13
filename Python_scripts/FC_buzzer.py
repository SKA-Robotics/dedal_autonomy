from pymavlink import mavutil

# Połączenie do pierwszego urządzenia
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')          # Łączenie się gdy proxy jest uruchomione
#master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)    # Łączenie się po USB serial bez proxy
master.wait_heartbeat()

#tune= "t200 o2 a8 a4"
#tune= "t200 o2 a8 a4 a8 a4"
target_system = master.target_system
target_component = master.target_component
#tune = "T200L16<cdefgab>cdefgab>c"
tune = "T140 o3 e2 p8 l4 e d e f e d c c2 p4 d2 e2 f2 f2 p16 f16 f f e d2 d2" #Meodyjka

# Podział melodii na dwie części, jeśli jest dłuższa niż 30 znaków
tune1 = tune[:30]
tune2 = tune[30:]

# Wysłanie wiadomości
master.mav.play_tune_send(
    target_system,
    target_component,
    tune1.encode(),
    tune2.encode()
)

print(f"Jesli zagrał to diała")


