#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import ImuData, AccelData, GyroData, EstimatorData, PossitionData
from example_interfaces.msg import String
from flask import Flask, render_template, jsonify, request, send_from_directory, abort, redirect, url_for, render_template_string
import threading
import os
from datetime import datetime, timedelta

# Ścieżka do katalogu danych
BASE_DIR = '/home/dron'  # Zmień na odpowiednią ścieżkę

# Uzyskanie ścieżki do katalogu z szablonami
template_dir = os.path.join(os.path.expanduser('~'), 'ws_controll/src/flask_pkg/flask_pkg/templates')

# Inicjalizacja aplikacji Flask z niestandardowym katalogiem szablonów
app = Flask(__name__, template_folder=template_dir)

data_buffer = [] 

# Inicjalizacja rclpy
rclpy.init()

# Tworzenie węzła ROS 2
class ServerNode(Node):
    def __init__(self):
        super().__init__('flask_server')
        self.subscription = self.create_subscription(
            EstimatorData,
            'estimation_data',
            self.listener_callback,
            100)
        self.subscription  # zapobiega usunięciu subskrypcji przez garbage collector
        self.counter = 0  # licznik do próbkowania
        self.publisher_ = self.create_publisher(String, 'flask_commands', 10)


    def listener_callback(self, msg):
        self.counter += 1
        if self.counter % 4 != 0:  # tylko co 10. wiadomość (czyli 10Hz)
            return  
        timestamp = datetime.utcnow()
        data_buffer.append(
            (timestamp, msg.possition.x, msg.possition.y, 
             msg.possition.z, msg.speed.x, 
             msg.speed.y, msg.speed.z, 
             msg.accel.x, msg.accel.y, msg.accel.z,
             msg.orientation.x, msg.orientation.y, msg.orientation.z,
             msg.raw_data.x, msg.raw_data.y, msg.raw_data.z
             ))
        cutoff = timestamp - timedelta(seconds=5)
        while data_buffer and data_buffer[0][0] < cutoff:
            data_buffer.pop(0)
        #self.get_logger().info('Odebrano: "%s"' % msg.timestamp)


    def publish_message(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Opublikowano: "{msg}"')




ros2_node = ServerNode()



# Uruchomienie węzła ROS 2 w osobnym wątku
def ros2_spin():
    rclpy.spin(ros2_node)

ros2_thread = threading.Thread(target=ros2_spin, daemon=True)
ros2_thread.start()


@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        print('POST')
        if request.form.get('reset') == 'Reset':
            ros2_node.publish_message('reset')
        elif request.form.get('calibrate') == 'Re-calibrate':
            ros2_node.publish_message('calibrate')
        elif request.form.get('start') == 'Start':
            ros2_node.publish_message('start_logging')
        elif request.form.get('restart') == 'Restart':
            ros2_node.publish_message('new_logging')
        elif request.form.get('stop') == 'Stop':
            ros2_node.publish_message('end_logging')
    return render_template('index.html')

@app.route('/files/<path:req_path>', methods=['GET'])
def dir_listing(req_path):
    abs_path = os.path.join(BASE_DIR, req_path)

    if not os.path.exists(abs_path):
        return abort(404)

    if os.path.isfile(abs_path):
        return send_from_directory(os.path.dirname(abs_path), os.path.basename(abs_path), as_attachment=True)

    files = os.listdir(abs_path)
    file_items = []

    for filename in files:
        full_path = os.path.join(abs_path, filename)
        rel_path = os.path.join(req_path, filename)

        if os.path.isfile(full_path):
            size = os.path.getsize(full_path)
            try:
                with open(full_path, 'r', encoding='utf-8') as f:
                    lines = sum(1 for _ in f)
            except:
                lines = "n/a"

            item = f'''
                <li>
                    <a href="/files/{rel_path}">{filename}</a> 
                    (rozmiar: {size} B, wiersze: {lines})
                    <form action="/delete" method="post" style="display:inline;">
                        <input type="hidden" name="file_path" value="{rel_path}">
                        <button type="submit">Usuń</button>
                    </form>
                </li>
            '''
        else:
            item = f'<li><a href="/files/{rel_path}">{filename}/</a> (katalog)</li>'

        file_items.append(item)

    return f'''
        <h1>Lista plików w: /{req_path}</h1>
        <ul>
            {''.join(file_items)}
        </ul>
    '''

@app.route('/delete', methods=['POST'])
def delete_file():
    file_path = request.form.get('file_path')
    abs_path = os.path.join(BASE_DIR, file_path)

    if os.path.isfile(abs_path):
        os.remove(abs_path)
        return redirect(url_for('dir_listing', req_path=os.path.dirname(file_path)))
    else:
        return abort(400, "To nie jest plik lub nie można usunąć.")

@app.route('/data')
def get_data():
    now = datetime.utcnow()
    recent = [(t, posx, posy, posz, spdx, spdy, spdz, accx, accy, accz, orientx, orienty, orientz, rawx, rawy, rawz) 
              for (t, posx, posy, posz, spdx, spdy, spdz, accx, accy, accz, orientx, orienty, orientz, rawx, rawy, rawz) in data_buffer]
    return jsonify([
        {
            'time': t.strftime('%H:%M:%S'),
            'position_x': posx,
            'position_y': posy,
            'position_z': posz,
            'speed_x': spdx,
            'speed_y': spdy,
            'speed_z': spdz,
            'accel_x': accx,
            'accel_y': accy,
            'accel_z': accz,
            'orientation_x': orientx,
            'orientation_y': orienty,
            'orientation_z': orientz,
            'raw_x': rawx,
            'raw_y': rawy,
            'raw_z': rawz
        } for (t, posx, posy, posz, spdx, spdy, spdz, accx, accy, accz, orientx, orienty, orientz, rawx, rawy, rawz) in recent
    ])

def main():
    app.run(debug=True,  host='0.0.0.0')

if __name__ == '__main__':
    main()
