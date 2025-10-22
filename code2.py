# ================================================================
#  SISTEM KONTROL FUZZY ADAPTIF KONVEYOR CERDAS (TRIMF SAJA)
#  Input  : FPS (Laju Frame) dan Confidence (Kepercayaan Deteksi)
#  Output : PWM (Kecepatan Konveyor) dan Delay (Jarak Aman Sortir)
#  Visualisasi: Menampilkan label fuzzy (LAMBAT, CUKUP, IDEAL, dll) di kamera
#  Penambahan: kirim perintah SERVO:HOLD ke Arduino saat Delay label == PENDEK
# ================================================================

import serial
import serial.tools.list_ports
import time
import cv2
from ultralytics import YOLO
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# ---------------------------------------------------------------
# 0. CARI DAN HUBUNGKAN KE ARDUINO OTOMATIS
# ---------------------------------------------------------------
def find_arduino():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in (p.description or "") or "CH340" in (p.description or ""):
            return p.device
    return None

arduino_port = find_arduino() or "COM8"
try:
    arduino = serial.Serial(arduino_port, 115200, timeout=1)
    time.sleep(2)
    print(f"✅ Terhubung ke Arduino di {arduino_port}")
except Exception as e:
    print(f"⚠ Gagal terhubung ke Arduino: {e}")
    arduino = None

# ---------------------------------------------------------------
# 1. LOAD YOLO MODEL
# ---------------------------------------------------------------
model = YOLO(r"D:\Tugas Mei\semester 7\Prak. Kontrol Cerdas\UTS\conveyor\code\train4\weights\best.pt")

# ---------------------------------------------------------------
# 2. SETUP KAMERA
# ---------------------------------------------------------------
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("❌ Kamera gagal dibuka! Coba ubah index ke 1 atau 2.")
    exit()
print("✅ Kamera aktif, mulai deteksi...")
prev_time = time.time()

# ---------------------------------------------------------------
# 3. DEFINISI VARIABEL FUZZY
# ---------------------------------------------------------------
fps = ctrl.Antecedent(np.arange(0, 16, 1), 'FPS')
confidence = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'Confidence')
pwm = ctrl.Consequent(np.arange(0, 256, 1), 'PWM')
delay = ctrl.Consequent(np.arange(50, 201, 1), 'Delay')

fps['LAMBAT'] = fuzz.trimf(fps.universe, [0, 0, 7])
fps['CUKUP']  = fuzz.trimf(fps.universe, [5, 7.5, 10])
fps['IDEAL']  = fuzz.trimf(fps.universe, [6, 15, 15])

confidence['RENDAH'] = fuzz.trimf(confidence.universe, [0.0, 0.0, 0.4])
confidence['SEDANG'] = fuzz.trimf(confidence.universe, [0.3, 0.5, 0.7])
confidence['TINGGI'] = fuzz.trimf(confidence.universe, [0.6, 1.0, 1.0])

pwm['MINIMAL']  = fuzz.trimf(pwm.universe, [0, 0, 70])
pwm['NORMAL']   = fuzz.trimf(pwm.universe, [80, 140, 200])
pwm['MAKSIMAL'] = fuzz.trimf(pwm.universe, [200, 255, 255])

delay['PANJANG'] = fuzz.trimf(delay.universe, [140, 200, 200])
delay['SEDANG']  = fuzz.trimf(delay.universe, [90, 125, 150])
delay['PENDEK']  = fuzz.trimf(delay.universe, [50, 50, 100])


# ---------------------------------------------------------------
# 4. ATURAN FUZZY
# ---------------------------------------------------------------
rules = [
    ctrl.Rule(fps['LAMBAT'] & confidence['RENDAH'], (pwm['MINIMAL'], delay['PANJANG'])),
    ctrl.Rule(fps['LAMBAT'] & confidence['SEDANG'], (pwm['MINIMAL'], delay['SEDANG'])),
    ctrl.Rule(fps['LAMBAT'] & confidence['TINGGI'], (pwm['NORMAL'], delay['PANJANG'])),

    ctrl.Rule(fps['CUKUP'] & confidence['RENDAH'], (pwm['MINIMAL'], delay['SEDANG'])),
    ctrl.Rule(fps['CUKUP'] & confidence['SEDANG'], (pwm['NORMAL'], delay['SEDANG'])),
    ctrl.Rule(fps['CUKUP'] & confidence['TINGGI'], (pwm['NORMAL'], delay['PENDEK'])),

    ctrl.Rule(fps['IDEAL'] & confidence['RENDAH'], (pwm['NORMAL'], delay['PANJANG'])),
    ctrl.Rule(fps['IDEAL'] & confidence['SEDANG'], (pwm['NORMAL'], delay['SEDANG'])),
    ctrl.Rule(fps['IDEAL'] & confidence['TINGGI'], (pwm['MAKSIMAL'], delay['PENDEK'])),
]

fuzzy_ctrl = ctrl.ControlSystem(rules)
konveyor = ctrl.ControlSystemSimulation(fuzzy_ctrl)

# ---------------------------------------------------------------
# 5. LABEL WARNA FUZZY
# ---------------------------------------------------------------
def label_fps(value):
    if value < 5: return "LAMBAT", (0, 0, 255)
    elif value < 10: return "CUKUP", (0, 255, 255)
    else: return "IDEAL", (0, 255, 0)

def label_conf(value):
    if value < 0.4: return "RENDAH", (0, 0, 255)
    elif value < 0.7: return "SEDANG", (0, 255, 255)
    else: return "TINGGI", (0, 255, 0)

def label_pwm(value):
    if value < 80: return "MINIMAL", (0, 0, 255)
    elif value < 200: return "NORMAL", (0, 255, 255)
    else: return "MAKSIMAL", (0, 255, 0)

def label_delay(value):
    if value < 90: return "PENDEK", (0, 255, 0)
    elif value < 140: return "SEDANG", (0, 255, 255)
    else: return "PANJANG", (0, 0, 255)

# ---------------------------------------------------------------
# 6. LOOP UTAMA
# ---------------------------------------------------------------
last_conf = 0.0

# cooldown supaya kita tidak mengirim SERVO:HOLD berkali-kali dalam 2 detik
last_servo_sent = 0.0
SERVO_COOLDOWN = 3.0  # detik (lebih panjang dari tahan servo 2 detik di Arduino)
MIN_DIST_THRESHOLD = 100  # pixel — batas minimal antar benda

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠ Frame tidak terbaca!")
        break

    curr_time = time.time()
    fps_now = 1 / (curr_time - prev_time + 1e-5)
    prev_time = curr_time

    results = model(frame, verbose=False)
    annotated = results[0].plot()

    conf_mean = 0
    centroids = []
    object_detected = False

    if len(results[0].boxes) > 0:
        conf_values = results[0].boxes.conf.cpu().numpy()
        conf_mean = float(np.mean(conf_values))
        object_detected = True
        
        # === Hitung centroid tiap objek ===
        boxes = results[0].boxes.xyxy.cpu().numpy()
        for box in boxes:
            x1, y1, x2, y2 = box[:4]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            centroids.append((cx, cy))
            cv2.circle(annotated, (cx, cy), 5, (255, 0, 255), -1)

         # === Hitung jarak antar centroid jika lebih dari 1 objek ===
        centroids.sort(key=lambda c: c[0])
        min_dist = None
        if len(centroids) > 1:
            dists = [abs(centroids[i + 1][0] - centroids[i][0]) for i in range(len(centroids) - 1)]
            min_dist = min(dists)
            centroid_history.append(min_dist)
            if len(centroid_history) > HISTORY_LEN:
                centroid_history.pop(0)
            min_dist = np.median(centroid_history)
            cv2.putText(annotated, f"Min Dist (smooth): {min_dist:.0f}px", (10, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            conf_mean = 0.3 * last_conf  # biar confidence nggak drop mendadak
            object_detected = False
            min_dist = 999
         
    konveyor.input['FPS'] = np.clip(fps_now, 0, 15)
    konveyor.input['Confidence'] = np.clip(conf_mean, 0, 1)
    konveyor.compute()

    pwm_fuzzy = konveyor.output['PWM']
    delay_out = konveyor.output['Delay']
    delay_label, delay_color = label_delay(delay_out)

    if object_detected and conf_mean > last_conf:
        pwm_out = pwm_fuzzy
    elif object_detected and conf_mean <= last_conf:
        pwm_out = max(pwm_fuzzy - 10, 0)
    else:
        pwm_out = 70

    last_conf = conf_mean

    fps_label, fps_color = label_fps(fps_now)
    conf_label, conf_color = label_conf(conf_mean)
    pwm_label, pwm_color = label_pwm(pwm_out)
    delay_label, delay_color = label_delay(delay_out)

    # --- KIRIM PWM KE ARDUINO (selalu kirim agar motor encoder tetap berjalan) ---
    try:
        if arduino and arduino.is_open:
            arduino.write(f"PWM:{int(pwm_out)}\n".encode())
    except Exception as e:
        print(f"⚠ Gagal kirim ke Arduino: {e}")

    # --- JIKA DELAY LABEL = PENDEK -> PERINTAH SERVO:HOLD ---
    # Pastikan ada objek (so detection meaningful) dan cooldown sudah lewat
        # --- LOGIKA SERVO ---
    now = time.time()
    if (
        object_detected
        and min_dist is not None
        and min_dist < MIN_DIST_THRESHOLD
        and delay_label == "PENDEK"
        and now - last_servo_sent > SERVO_COOLDOWN
    ):
        arduino.write(b"SERVO:HOLD\n")
        print(f">> Servo aktif: delay PENDEK + jarak {min_dist:.0f}px")
        last_servo_sent = now
            
    # --- Tampilkan di terminal + overlay ke frame ---
    print(f"FPS: {fps_now:.2f} ({fps_label}) | Conf: {conf_mean:.2f} ({conf_label}) | "
          f"PWM: {pwm_out:.0f} ({pwm_label}) | Delay: {delay_out:.0f} ms ({delay_label})")

    cv2.putText(annotated, f"FPS:{fps_now:.1f} ({fps_label})", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, fps_color, 2)
    cv2.putText(annotated, f"Conf:{conf_mean:.2f} ({conf_label})", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, conf_color, 2)
    cv2.putText(annotated, f"PWM:{pwm_out:.0f} ({pwm_label})", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, pwm_color, 2)
    cv2.putText(annotated, f"Delay:{delay_out:.0f} ms ({delay_label})", (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, delay_color, 2)

    cv2.imshow("Fuzzy Adaptive Conveyor | Visualisasi Warna", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ---------------------------------------------------------------
# 7. CLEANUP
# ---------------------------------------------------------------
cap.release()
cv2.destroyAllWindows()
if arduino and arduino.is_open:
    arduino.close()
print("🚀 Program selesai.")
