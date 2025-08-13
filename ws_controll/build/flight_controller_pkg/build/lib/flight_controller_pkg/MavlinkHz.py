# import time as tm
# from datetime import datetime, timedelta # Import datetime for time control
# import os # Import os for creating csv file
# import csv # Import csv for save to file
# from pymavlink import mavutil # Import mavutil for Mavlink comunikation
# import pandas as pd
# import numpy as np

# from sklearn.linear_model import LinearRegression
# from scipy.signal import kaiserord, firwin, lfilter, buttord

# # G_force = 9.80665
# G_force = 9.81228 # Warsaw g value

# # ==============================================================================
# # -- MavLink ---------------------------------------------------------------
# # ==============================================================================
# class MavLinkConfigurator:
#     def __init__(self, com= 'COM5', baud= 115200):
#         # Create the connection
#         self.master = mavutil.mavlink_connection(com, baud)
#         # Wait a heartbeat before sending commands
#         self.master.wait_heartbeat()
        
#     def end_connection(self):
#         self.master.close()
#         tm.sleep(1)
    
#     def request_message(self, mess_type= 'HIGHRES_IMU'):
#         try:
#             mes = self.master.recv_match(type= mess_type, blocking= True)
#             if mes.msgname == mess_type:
#                 return mes
#             else:
#                 return self.request_message(mess_type)
#         except:
#             return None

#     def request_message_interval(self, message_id: int, frequency_hz: float):
#         self.master.mav.command_long_send(
#             self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
#             message_id, # The MAVLink message ID
#             1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
#             0, 0, 0, 0, # Unused parameters
#             0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
#         )

#     def tune(self, tune= "t200 o2 a8 a4"):
#         target_system = self.master.target_system
#         target_component = self.master.target_component
#         #tune = "T200L16<cdefgab>cdefgab>c"
#         #tune = "T140 o3 e2 p8 l4 e d e f e d c c2 p4 d2 e2 f2 f2 p16 f16 f f e d2 d2" #Meodyjka

#         # Podział melodii na dwie części, jeśli jest dłuższa niż 30 znaków
#         tune1 = tune[:30]
#         tune2 = tune[30:]

#         # Wysłanie wiadomości
#         self.master.mav.play_tune_send(
#             target_system,
#             target_component,
#             tune1.encode(),
#             tune2.encode()
#         )

#     def show_me_everything(self):
#         while True:
#             mes = self.master.recv_msg()
#             if mes:
#                 print(mes)               
#             tm.sleep(0.001)

# # ==============================================================================
# # -- Lowpass filter ---------------------------------------------------------------
# # ==============================================================================

# class RealTimeFilter:
#     def __init__(self):
#         # Parametry filtru 
#         self.passband = 10e-20 # Znormalizowana częstotliwość graniczna pasma przepustowego
#         self.stopband = 0.1 # Znormalizowana częstotliwość graniczna pasma zaporowego
#         self.max_ripple = 0.1 # Maksymalne tętnienie w paśmie przepustowym (dB)
#         self.min_attenuation = 50 # Minimalne tłumienie w paśmie zaporowym (dB)
        
#         # Oblicz współczynniki filtru
#         transition_width = self.stopband - self.passband
#         self.n, alfa = kaiserord(self.min_attenuation, transition_width)
#         self.taps = firwin(self.n, self.passband, window=('kaiser', alfa), pass_zero='lowpass')
        
#         # Bufor danych (przechowuje N ostatnich próbek)
#         self.buffer = []
#         for i in range(0,6):
#             if i != 2:
#                 self.buffer.append(np.zeros(len(self.taps) - 1))  # stan początkowy
#             else:
#                 self.buffer.append(np.zeros(len(self.taps) - 1) - G_force)

        
#         self.filtered_sample = [0, 0, G_force, 0, 0, 0]

#     def temp_offsets(self, IMU_data, temp):
#         IMU_data[0][0] = IMU_data[0][0] - temp * 50 / 100000 + 0.022
#         IMU_data[0][1] = IMU_data[0][1] - temp * 150 / 100000 + 0.07
#         IMU_data[0][2] = IMU_data[0][2] + temp * 700 / 100000 - 0.32
#         IMU_data[1][0] = IMU_data[1][0] - temp * 0.275 / 100000 + 0.0
#         IMU_data[1][1] = IMU_data[1][1] - temp * 0.165 / 100000 + 0.0
#         IMU_data[1][2] = IMU_data[1][2] - temp * 0.125 / 100000 + 0.0
#         return IMU_data

#     def process_sample(self, new_sample):
#         for i in range(0,6):
#             # Aktualizuj bufor: dodaj nową próbkę na koniec
#             input_signal = np.concatenate([self.buffer[i], [new_sample[i]]])
            
#             # Filtruj (użyj tylko ostatniej próbki wyjściowej)
#             output_signal = lfilter(self.taps, 1.0, input_signal)
#             self.filtered_sample[i] = output_signal[-1]
            
#             # Zaktualizuj bufor dla następnego cyklu
#             self.buffer[i] = input_signal[1:]
        
#         return self.filtered_sample

#     def filter_data(self, IMU_data):

#         temp_struct = [IMU_data[0][0], IMU_data[0][1], IMU_data[0][2], IMU_data[1][0], IMU_data[1][1], IMU_data[1][2]]
#         temp_struct = self.process_sample(temp_struct)
#         IMU_data[0][0] = temp_struct[0]
#         IMU_data[0][1] = temp_struct[1]
#         IMU_data[0][2] = temp_struct[2]
#         IMU_data[1][0] = temp_struct[3]
#         IMU_data[1][1] = temp_struct[4]
#         IMU_data[1][2] = temp_struct[5]
#         return IMU_data

#     def filter_row(self):
#         print(f"Rząd filtru IIR: {self.n}")

#     def load_IMU_CSV(self, file= "imu_data.csv"):
#         data = pd.read_csv(file)
#         if file == "static/imu_data.csv":
#             self.regresion(data)
#         deadband = 0
#         accelerometer = np.array(data.iloc[deadband:, 1:4])
#         gyroscope = np.array(data.iloc[deadband:, 4:7])
#         timestamp = np.array(data.iloc[deadband:, 0])
#         temperature = np.array(data.iloc[deadband:, 11])
#         sensor_data = [accelerometer, gyroscope, timestamp, temperature]
#         return sensor_data

# # ==============================================================================
# # -- Calibrator ---------------------------------------------------------------
# # ==============================================================================

# class CalibrateIMU:
#     def __init__(self):
#         self.connection = MavLinkConfigurator()
#         self.save_temp = DataLogger()
#         self.filter = RealTimeFilter()
        

#     def do_magic(self):
#         self.colect_data()
#         self.connection.end_connection()
#         offsets = self.calculate_offsets()
#         coeficients = self.regresion()
#         scaled_offsets_1 = self.calculate_scaled_offsets('imu2_calibration_data.csv')
#         scaled_offsets_2 = self.calculate_scaled_offsets('imu3_calibration_data.csv')
#         self.save_temp.remove_file('imu_calibration_data.csv')
#         self.save_temp.remove_file('imu2_calibration_data.csv')
#         self.save_temp.remove_file('imu3_calibration_data.csv')
#         return offsets, scaled_offsets_1, scaled_offsets_2, coeficients

#     def regresion(self):
#         # Utwórz model regresji
#         model = LinearRegression()

#         coeficients = []
#         struct = [[],[],[],[],[],[]]

#         df = pd.read_csv('imu_calibration_data.csv')
        
#         for i in range(0,len(df.iloc[:,0])):
#             IMU_data = [[df.iloc[i, 1], df.iloc[i, 2], df.iloc[i, 3]],[df.iloc[i, 4], df.iloc[i, 5], df.iloc[i, 6]]]
#             IMU_data = self.filter.filter_data(IMU_data)
#             struct[0].append(IMU_data[0][0])
#             struct[1].append(IMU_data[0][1])
#             struct[2].append(IMU_data[0][2])
#             struct[3].append(IMU_data[1][0])
#             struct[4].append(IMU_data[1][1])
#             struct[5].append(IMU_data[1][2])


#         temperature = df.iloc[:, 11]
#         temperature = np.array(temperature).reshape(-1, 1)

#         for i in range(0,6):
#             #y = df.iloc[:, i]
#             y=struct[i]
#             # Dopasuj model do danych
#             model.fit(temperature, y)

#             # Współczynniki regresji
#             intercept = model.intercept_  # Wyraz wolny (b)
#             slope = model.coef_[0]        # Nachylenie (a)

#             coeficients.append(slope)
#             coeficients.append(intercept)

#             #print(f"Funkcja liniowa: y = {slope}x + {intercept}")
#         coeficients[5] -= G_force
#         return coeficients

#     def colect_data(self):
#         freq = 200
#         data_rows = 10*freq

#         self.connection.request_message_interval(105, freq) # HIGHRES_IMU request
#         self.connection.request_message_interval(116, freq) # SCALED_IMU2 request
#         self.connection.request_message_interval(129, freq) # SCALED_IMU3 request
#         self.connection.tune()

#         self.save_temp.remove_file('imu_calibration_data.csv')
#         self.save_temp.remove_file('imu2_calibration_data.csv')
#         self.save_temp.remove_file('imu3_calibration_data.csv')

#         timestamp_last_mes = datetime.now()
#         period = timedelta(milliseconds=1/freq)

#         counter = 0
#         percent = 0.1
#         while True:
#             time_delta = datetime.now() - timestamp_last_mes
#             if time_delta > period:
#                 mes = self.connection.request_message()
                
#                 if mes.temperature > 40:
#                     self.save_temp.save_IMU_data(mes, 'imu_calibration_data.csv')

#                     mes = self.connection.request_message("SCALED_IMU2")
                
#                     self.save_temp.save_IMU_data(mes, 'imu2_calibration_data.csv')

#                     mes = self.connection.request_message("SCALED_IMU3")
                
#                     self.save_temp.save_IMU_data(mes, 'imu3_calibration_data.csv')

#                     if mes != None:
#                         counter += 1

#                     timestamp_last_mes = datetime.now()

#                     if counter >= data_rows*percent:
#                         percent += 0.1
#                         #self.connection.tune("t200 o1 c16")
#                 else:
#                     timestamp_last_mes = timestamp_last_mes+period*1000
#                     print("Heating up:", round(mes.temperature, 2), "C deg")

#             if counter == data_rows:
#                 break
#         self.connection.tune()

#     def calculate_offsets(self):
#         # Wczytanie danych z pliku CSV
#         df = pd.read_csv('imu_calibration_data.csv')

#         accx = df.iloc[:, 1]
#         accy = df.iloc[:, 2]
#         accz = df.iloc[:, 3]
#         xgyro = df.iloc[:, 4]
#         ygyro = df.iloc[:, 5]
#         zgyro = df.iloc[:, 6]

#         accx_mean = accx.mean()
#         accy_mean = accy.mean()
#         accz_mean = accz.mean()
#         xgyro_mean = xgyro.mean()
#         ygyro_mean = ygyro.mean()
#         zgyro_mean = zgyro.mean()

#         accz_mean -= G_force

#         return [accx_mean, accy_mean, accz_mean, xgyro_mean, ygyro_mean, zgyro_mean]

#     def calculate_scaled_offsets(self, file_patch):
#         # Wczytanie danych z pliku CSV
#         file = pd.read_csv(file_patch)
#         accx = file.iloc[:, 1]
#         accy = file.iloc[:, 2]
#         accz = file.iloc[:, 3]
#         xgyro = file.iloc[:, 4]
#         ygyro = file.iloc[:, 5]
#         zgyro = file.iloc[:, 6]
#         accx_mean = accx.mean()
#         accy_mean = accy.mean()
#         accz_mean = accz.mean()
#         xgyro_mean = xgyro.mean()
#         ygyro_mean = ygyro.mean()
#         zgyro_mean = zgyro.mean()

#         accz_mean += G_force
        
#         return [accx_mean, accy_mean, accz_mean, xgyro_mean, ygyro_mean, zgyro_mean]

# # ==============================================================================
# # -- CSV ---------------------------------------------------------------
# # ==============================================================================

# class DataLogger:
#     def __init__(self):
#         pass
        

#     def add_IMU_headers(self, filename):
#         # Dodanie nagłówków do pliku tylko przy pierwszym zapisie
#         with open(filename, mode= 'a', newline= '') as file:
#             writer = csv.writer(file)

#             writer.writerow([
#             'Timestamp',
#             'Accelerometer_X', 'Accelerometer_Y', 'Accelerometer_Z', 
#             'Gyroscope_X', 'Gyroscope_Y', 'Gyroscope_Z', 
#             'Magnetometer_X', 'Magnetometer_Y', 'Magnetometer_Z',
#             'Abs_Pressure',
#             'Temperature'
#             ])

#     def remove_file(self, filename):
#         # Jeśli plik już istnieje, usuń go, aby nadpisać od nowa
#         if os.path.isfile(filename):
#             os.remove(filename)

#     def save_IMU_data(self, mes, filename, offsets= [0.0,0,0,0,0,0], coeficients= [0.0,0,0,0,0,0,0.0,0,0,0,0,0]):

#         # Otwarcie pliku w trybie dołączania (append)
#         if(filename == "imu_data.csv" or filename == "imu_calibration_data.csv"):
#             temp = mes.temperature
#             with open(filename, mode='a', newline='') as file:
#                 writer = csv.writer(file)
#                 # Zapisanie danych IMU w formacie (x, y, z)
#                 writer.writerow([
#                     mes.time_usec,
#                     mes.xacc - offsets[0],
#                     mes.yacc - offsets[1],
#                     mes.zacc - offsets[2],
#                     mes.xgyro - offsets[3],
#                     mes.ygyro - offsets[4],
#                     mes.zgyro - offsets[5],
#                     mes.xmag,
#                     mes.ymag,
#                     mes.zmag,
#                     mes.abs_pressure,
#                     mes.temperature
#                 ])

#                 # writer.writerow([
#                 #     mes.time_usec,
#                 #     mes.xacc - coeficients[0]*temp - coeficients[1],
#                 #     mes.yacc - coeficients[2]*temp - coeficients[3],
#                 #     mes.zacc - coeficients[4]*temp - coeficients[5],
#                 #     mes.xgyro - coeficients[6]*temp - coeficients[7],
#                 #     mes.ygyro - coeficients[8]*temp - coeficients[9],
#                 #     mes.zgyro - coeficients[10]*temp - coeficients[11],
#                 #     mes.xmag,
#                 #     mes.ymag,
#                 #     mes.zmag,
#                 #     mes.abs_pressure,
#                 #     mes.temperature
#                 # ])
#         elif(filename == "imu2_data.csv" or filename == "imu2_calibration_data.csv"
#              or filename == "imu3_data.csv" or filename == "imu3_calibration_data.csv"):
#             with open(filename, mode='a', newline='') as file:
#                 writer = csv.writer(file)
#                 # Zapisanie danych IMU w formacie (x, y, z)
#                 writer.writerow([
#                     mes.time_boot_ms,
#                     mes.xacc/100 - offsets[0],
#                     mes.yacc/100 - offsets[1],
#                     mes.zacc/100 - offsets[2],
#                     mes.xgyro/100 - offsets[3],
#                     mes.ygyro/100 - offsets[4],
#                     mes.zgyro/100 - offsets[5],
#                     mes.xmag,
#                     mes.ymag,
#                     mes.zmag,
#                     mes.temperature/100
#                 ])

# # ==============================================================================
# # -- DataColection ---------------------------------------------------------------
# # ==============================================================================

# class MainData:
#     def __init__(self):
#         self.connection = MavLinkConfigurator()
#         self.save_file = DataLogger()
#         self.temp_storage = []

#     def public_csv(self):
#         filename = 'imu_data.csv'
#         self.save_file.remove_file(filename)
#         self.save_file.add_IMU_headers(filename)
#         filename = 'imu2_data.csv'
#         self.save_file.remove_file(filename)
#         self.save_file.add_IMU_headers(filename)
#         filename = 'imu3_data.csv'
#         self.save_file.remove_file(filename)
#         self.save_file.add_IMU_headers(filename)


#     def do_magic(self, offsets= [0,0,0,0,0,0], scaled_offsets_1= [0,0,0,0,0,0], scaled_offsets_2= [0,0,0,0,0,0], coeficients= [0,0,0,0,0,0,0,0,0,0,0,0]):
#         duration = 1*20
#         freq = 100
#         data_rows = duration * freq

#         self.public_csv()

#         # self.connection.request_message_interval(27, 1) # RAW_IMU request
#         self.connection.request_message_interval(116, freq) # SCALED_IMU2 request
#         self.connection.request_message_interval(129, freq) # SCALED_IMU3 request
#         self.connection.request_message_interval(105, freq) # HIGHRES_IMU request at 100 Hz

#         self.connection.tune()
#         # self.connection.show_me_everything()
        
#         timestamp_last_mes = datetime.now()
#         period = timedelta(milliseconds=1/freq)
#         timestamp1 = datetime.now()


#         counter = 0
#         percent = 0.1
#         while True:
#             time_delta = datetime.now() - timestamp_last_mes
#             if time_delta > period:
#                 mes = self.connection.request_message()
                
#                 self.save_file.save_IMU_data(mes, 'imu_data.csv', offsets, coeficients)

#                 mes = self.connection.request_message("SCALED_IMU2")
                
#                 self.save_file.save_IMU_data(mes, 'imu2_data.csv', scaled_offsets_1)

#                 mes = self.connection.request_message("SCALED_IMU3")
                
#                 self.save_file.save_IMU_data(mes, 'imu3_data.csv', scaled_offsets_2)

#                 if mes != None:
#                     counter += 1

#                 if counter >= data_rows*percent:
#                     #self.connection.tune("t200 o1 c16")
#                     print("Pozostało", round(duration - percent*duration,0), "s")
#                     percent += 0.1

#                 timestamp_last_mes = datetime.now()

#             if counter == data_rows:
#                 break

#         timestamp2 = datetime.now()
#         self.connection.tune()
#         print("Koniec pomiaru")
#         print(f"Czas na {data_rows} wiadomości {timestamp2-timestamp1}")

#         self.connection.end_connection()
#         # connection.show_me_everything() # uncomment to display all messages to terminal

# # ==============================================================================
# # -- main ---------------------------------------------------------------
# # ==============================================================================

# def main():
#     # connection = MavLinkConfigurator()
#     # connection.tune("T140 o3 e2 p8 l4 e d e f e d c2 c2 p4 d2 e2 f2 f2 p16 f16 f f e d2 d2")  #Meodyjka
    
#     calibrator = CalibrateIMU()
#     print("Zbieranie danych do kalibracji")
#     offsets, scaled_offsets_1, scaled_offsets_2, coeficients = calibrator.do_magic()
#     print("Odchyłki:", offsets)
#     print("Odchyłki 2:", scaled_offsets_1)
#     print("Odchyłki 3:", scaled_offsets_2)

#     print("Rozpoczynanie pomiaru")
#     public_data = MainData()
#     public_data.do_magic(offsets, scaled_offsets_1, scaled_offsets_2, coeficients)


# if __name__ == '__main__':

#     main()