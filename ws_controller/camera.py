from flask import Flask, Response, render_template_string, send_file
import depthai as dai
import cv2
import threading
import time
import io

# --- Konfiguracja kamery / DepthAI ---
pipeline = dai.Pipeline()

cam = pipeline.createColorCamera()
# Lekkie 16:9 pod MJPEG; możesz podnieść, np. 1280x720
cam.setPreviewSize(640, 360)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setFps(30)

xout = pipeline.createXLinkOut()
xout.setStreamName("preview")
cam.preview.link(xout.input)

device = dai.Device(pipeline)
q = device.getOutputQueue(name="preview", maxSize=4, blocking=False)

# --- Bufor ramki współdzielony ze strumieniem Flask ---
last_frame = None
lock = threading.Lock()
running = True

def grab_frames():
    """Wątek pobierający klatki z OAK i aktualizujący last_frame."""
    global last_frame, running
    while running:
        pkt = q.get()  # blokuje do momentu pojawienia się klatki
        frame = pkt.getCvFrame()
        with lock:
            last_frame = frame
        # lekki oddech – obniża zużycie CPU, można usunąć
        time.sleep(0.001)

t = threading.Thread(target=grab_frames, daemon=True)
t.start()

# --- Aplikacja Flask ---
app = Flask(__name__)

INDEX_HTML = """
<!doctype html>
<html lang="pl">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>OAK-D-Lite Stream</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 0; padding: 24px; background:#0b0d10; color:#e8eaed; }
    .wrap { max-width: 960px; margin: 0 auto; }
    img { width: 100%; height: auto; border-radius: 12px; display:block; }
    header { display:flex; align-items:center; justify-content:space-between; margin-bottom:12px; }
    a.btn { background:#1a73e8; color:white; padding:8px 12px; border-radius:8px; text-decoration:none; }
    a.btn:hover { opacity:.9; }
  </style>
</head>
<body>
  <div class="wrap">
    <header>
      <h1>OAK-D-Lite → Flask (MJPEG)</h1>
      <div>
        <a class="btn" href="/snapshot.jpg" target="_blank">Zrób snapshot</a>
      </div>
    </header>
    <img src="/video_feed" alt="Live stream" />
    <p style="opacity:.8;margin-top:12px">Naciśnij link „Zrób snapshot”, aby pobrać aktualną klatkę.</p>
  </div>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(INDEX_HTML)

def mjpeg_generator():
    """Zwraca kolejne klatki JPEG w formacie multipart/x-mixed-replace."""
    # Uwaga: jeśli nie ma jeszcze ramki, poczekajmy chwilę
    while True:
        with lock:
            frame = None if last_frame is None else last_frame.copy()
        if frame is None:
            time.sleep(0.01)
            continue
        # Kodowanie do JPEG (90% jakości – balans CPU/jakość)
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            continue
        chunk = jpg.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(chunk)).encode() + b"\r\n\r\n" +
               chunk + b"\r\n")

@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/snapshot.jpg")
def snapshot():
    # Jednorazowy zrzut aktualnej klatki
    with lock:
        frame = None if last_frame is None else last_frame.copy()
    if frame is None:
        # Gdyby ktoś wejdzie bardzo wcześnie – krótki wait
        time.sleep(0.05)
        with lock:
            frame = None if last_frame is None else last_frame.copy()
    if frame is None:
        # Brak klatki – zwróć pusty JPEG
        buf = io.BytesIO()
        cv2.imencode(".jpg", (255 * (0 * 0)).astype("uint8"))
        return send_file(buf, mimetype="image/jpeg")
    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    buf = io.BytesIO(jpg.tobytes())
    buf.seek(0)
    return send_file(buf, mimetype="image/jpeg")

def shutdown():
    global running
    running = False
    try:
        t.join(timeout=1.0)
    except:
        pass
    device.close()

import atexit
atexit.register(shutdown)

if __name__ == "__main__":
    # host=0.0.0.0 żeby było dostępne w sieci LAN
    app.run(host="0.0.0.0", port=5001, threaded=True)
