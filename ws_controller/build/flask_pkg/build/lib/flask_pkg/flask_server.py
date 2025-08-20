import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from custom_msgs.msg import DroneStatus, GeoData
from example_interfaces.msg import String
from flask import Flask, render_template, jsonify, request, make_response
import mimetypes
import threading
import os
from datetime import datetime, timedelta
import csv
import io
import uuid


# Ścieżka do katalogu danych (niewykorzystywana tutaj, zostawiamy jak było)
BASE_DIR = '/home/dron'  # Zmień na odpowiednią ścieżkę

# Katalog szablonów
template_dir = os.path.join(os.path.expanduser('~'), 'ws_controller/src/flask_pkg/flask_pkg/templates')
static_dir = os.path.join(os.path.expanduser('~'), 'ws_controller/src/flask_pkg/flask_pkg/static')

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
            qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(String, 'flask_commands', 10)
        self.geo_pub = self.create_publisher(GeoData, 'geo_points', 10)


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
            cutoff = timestamp - timedelta(seconds=60)
            # Zostaw tylko ostatnie 60 s danych
            while data_buffer and data_buffer[0][0] < cutoff:
                data_buffer.pop(0)
        #self.get_logger().info(f'Odebrano nap. baterii: {msg.battery_voltage}')

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
        print("Obebrano POST")
        if request.form.get('logging') == 'Logging':
            ros2_node.publish_message('start_logging')

        elif request.form.get('arm') == 'Arm':
            ros2_node.publish_message('set_arm')
        elif request.form.get('disarm') == 'Disarm':
            ros2_node.publish_message('set_disarm')

        elif request.form.get('land') == 'Land':
            ros2_node.publish_message('land_now')
        elif request.form.get('stabilize') == 'Stabilize':
            ros2_node.publish_message('stabilize')

        elif request.form.get('start_hower') == 'Start_Hower':
            ros2_node.publish_message('start_hower')
        elif request.form.get('cancel') == 'Cancel':
            ros2_node.publish_message('cancel_mission')

        elif request.form.get('play_barka') == 'Play_barka':
            ros2_node.publish_message('play_Barka')

        elif request.form.get('remove_geo') == 'Remove_geo':
            ros2_node.publish_message('remove_geo')
        elif request.form.get('set_geo') == 'Set_geo':
            ros2_node.publish_message('set_geo')

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

@app.route('/api/upload-csv', methods=['POST'])
def upload_csv():
    """
    Przyjmuje plik CSV (multipart/form-data: file=<csv>),
    parsuje kolumny lat, lon, [alt] i publikuje je na topic 'geo_points' jako GeoData.
    Zwraca liczbę opublikowanych punktów.
    """
    if 'file' not in request.files:
        return jsonify({'error': 'Brak pliku w żądaniu (pole "file").'}), 400

    file = request.files['file']
    if not file.filename.lower().endswith('.csv'):
        return jsonify({'error': 'Dozwolone są tylko pliki .csv'}), 400

    # Opcjonalnie zapisz kopię na dysk (dla logów / re-użycia)
    uploads_dir = '/tmp/uploads'
    os.makedirs(uploads_dir, exist_ok=True)
    safe_name = f"{datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')}_{uuid.uuid4().hex}.csv"
    save_path = os.path.join(uploads_dir, safe_name)
    file.stream.seek(0)
    file.save(save_path)

    # Wczytaj do pamięci do parsowania
    file.stream.seek(0)
    file.stream.flush()
    file.stream.seek(0)
    content = file.read().decode('utf-8', errors='replace')
    reader = csv.DictReader(io.StringIO(content))

    count = 0
    errors = 0

    required = {'lat', 'lon'}
    # Sprawdź czy nagłówki mają przynajmniej lat, lon
    if not required.issubset({h.strip().lower() for h in reader.fieldnames or []}):
        return jsonify({'error': 'CSV musi zawierać nagłówki kolumn: lat, lon (alt opcjonalne).'}), 400

    for i, row in enumerate(reader, start=1):
        try:
            lat = float(str(row.get('lat', '')).strip())
            lon = float(str(row.get('lon', '')).strip())
            alt_str = row.get('alt', '')
            alt = float(str(alt_str).strip()) if (alt_str is not None and str(alt_str).strip() != '') else 0.0

            msg = GeoData()
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            ros2_node.geo_pub.publish(msg)
            count += 1
        except Exception:
            errors += 1
            continue

    return jsonify({
        'ok': True,
        'points': count,
        'errors': errors,
        'filename': os.path.basename(save_path)
    }), 200


def main():
    # host=0.0.0.0 aby dostęp był z sieci; debug opcjonalny
    app.run(debug=True, use_reloader=False, host='0.0.0.0')


if __name__ == '__main__':
    main()