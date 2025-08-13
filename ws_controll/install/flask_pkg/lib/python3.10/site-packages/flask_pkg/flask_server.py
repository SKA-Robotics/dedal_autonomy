import rclpy
from rclpy.node import Node
from custom_msgs.msg import DroneStatus, GeoData
from example_interfaces.msg import String
from flask import Flask, render_template, jsonify, request, make_response
import mimetypes
import threading
import os
from datetime import datetime, timedelta

# Ścieżka do katalogu danych (niewykorzystywana tutaj, zostawiamy jak było)
BASE_DIR = '/home/dron'  # Zmień na odpowiednią ścieżkę

# Katalog szablonów
template_dir = os.path.join(os.path.expanduser('~'), 'ws_controll/src/flask_pkg/flask_pkg/templates')
static_dir = os.path.join(os.path.expanduser('~'), 'ws_controll/src/flask_pkg/flask_pkg/static')

# Flask
app = Flask(__name__, template_folder=template_dir)
app = Flask(
    __name__,
    template_folder=template_dir,
    static_folder=static_dir,
    static_url_path='/static'
)

# Bufor i blokada współbieżna (NEW)
data_buffer = []
data_lock = threading.Lock()

# Inicjalizacja rclpy
rclpy.init(args=None)

class ServerNode(Node):
    def __init__(self):
        super().__init__('flask_server')
        self.subscription = self.create_subscription(
            DroneStatus,
            'drone_status',
            self.listener_callback,
            100
        )
        self.publisher_ = self.create_publisher(String, 'flask_commands', 10)

    def listener_callback(self, msg: DroneStatus):
        timestamp = datetime.utcnow()
        sample = (
            timestamp,
            msg.is_autonomy_active,
            msg.battery_voltage,
            msg.ekf_position.latitude,
            msg.ekf_position.longitude,
            msg.ekf_position.altitude,
        )
        # Zapis z lockiem (NEW)
        with data_lock:
            data_buffer.append(sample)
            print(len(data_buffer))
            cutoff = timestamp - timedelta(seconds=60)
            # Zostaw tylko ostatnie 60 s danych
            while data_buffer and data_buffer[0][0] < cutoff:
                data_buffer.pop(0)
        self.get_logger().info(f'Odebrano nap. baterii: {msg.battery_voltage}')

    def publish_message(self, command: str):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Opublikowano komendę: "{command}"')

ros2_node = ServerNode()

# Wątek ROS2

def ros2_spin():
    rclpy.spin(ros2_node)

ros2_thread = threading.Thread(target=ros2_spin, daemon=True)
ros2_thread.start()

# =============== ROUTES ===============
@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        if request.form.get('reset') == 'Reset':
            ros2_node.publish_message('reset')
        elif request.form.get('start') == 'Start':
            ros2_node.publish_message('start_logging')
    return render_template('index.html')

# NEW: API – zwróć okno danych (ostatnie ~60 s)
@app.route('/api/data')
def api_data():
    with data_lock:
        payload = [
            {
                't': ts.isoformat() + 'Z',
                'autonomy': bool(auth),
                'voltage': float(voltage),
                'lat': float(lat),
                'lon': float(lon),
                'alt': float(alt),
            }
            for (ts, auth, voltage, lat, lon, alt) in data_buffer
        ]
    return jsonify(payload)

# NEW: API – ostatnia próbka
@app.route('/api/latest')
def api_latest():
    with data_lock:
        if not data_buffer:
            return jsonify({'available': False}), 200
        ts, auth, voltage, lat, lon, alt = data_buffer[-1]
        return jsonify({
            'available': True,
            't': ts.isoformat() + 'Z',
            'autonomy': bool(auth),
            'voltage': float(voltage),
            'lat': float(lat),
            'lon': float(lon),
            'alt': float(alt),
        })

@app.route('/api/health')
def api_health():
    with data_lock:
        return jsonify({'buffer_len': len(data_buffer)})

@app.route('/api/mock')
def api_mock():
    ts = datetime.utcnow()
    sample = (ts, True, 11.9, 52.2297, 21.0122, 120.0)  # przykładowe dane
    with data_lock:
        data_buffer.append(sample)
    return jsonify({'ok': True})

@app.route('/sw.js')
def service_worker():
    # Serwujemy SW z korzenia, żeby miał scope '/'
    sw_path = os.path.join(os.path.dirname(__file__), 'sw.js')
    if not os.path.exists(sw_path):
        return "", 404
    with open(sw_path, 'rb') as f:
        resp = make_response(f.read())
        resp.headers['Content-Type'] = 'application/javascript'
        return resp


def main():
    # host=0.0.0.0 aby dostęp był z sieci; debug opcjonalny
    app.run(debug=True, use_reloader=False, host='0.0.0.0')


if __name__ == '__main__':
    main()