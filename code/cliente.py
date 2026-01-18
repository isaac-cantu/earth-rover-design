import socket
import struct
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from PIL import Image, ImageTk
import time
import json
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math

# ==================== CONFIGURACIÓN ====================
SERVER_IP = "----"  # CAMBIAR A TU IP
UDP_PORT = 50000
TCP_PORT_VIDEO_FRONTAL = 50001
TCP_PORT_VIDEO_SUPERIOR = 50002
TCP_PORT_STATUS = 50003
BUFFER_SIZE = 4096

# ==================== DETECCIÓN DE MARCADORES ====================
RANGOS_HU = {
    "Cruz": {
        1: (0.68, 0.75), 2: (2.7, 5.1), 3: (0, 6.5), 4: (5, 7.5),
        5: (-14, 15), 6: (-12, 11), 7: (-13.5, 13)
    },
    "T": {
        1: (0.3, 0.7), 2: (0.5, 4), 3: (1, 2), 4: (1, 4),
        5: (-10, 7), 6: (-5, 6), 7: (-8, 7)
    },
    "Circulo": {
        1: (0.72, 0.81), 2: (3.35, 5.1), 3: (4.5, 6.5), 4: (5.2, 8.4),
        5: (-17, 18), 6: (-14, 13), 7: (-17, 17.5)
    },
    "Triangulo": {
        1: (0.6, 1), 2: (2, 4), 3: (2, 4), 4: (3.5, 6),
        5: (6.5, 10), 6: (4.8, 10), 7: (-10, 10)
    },
    "Cuadrado": {
        1: (0.70, 0.89), 2: (2, 6.5), 3: (3, 7), 4: (4, 7.9),
        5: (-15, 15), 6: (-11, 11), 7: (-15.7, 14)
    }
}

LOWER_ORANGE = np.array([8, 150, 180])
UPPER_ORANGE = np.array([18, 255, 255])
LOWER_GREEN = np.array([35, 120, 150])
UPPER_GREEN = np.array([85, 255, 255])
LOWER_YELLOW = np.array([22, 140, 150])
UPPER_YELLOW = np.array([32, 255, 255])

# ==================== VARIABLES GLOBALES ====================
running = True
frame_frontal = None
frame_superior = None
estado_rover = {}
estado_lock = threading.Lock()

# Odometría
class Odometria:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.historial_x = deque(maxlen=200)
        self.historial_y = deque(maxlen=200)
        self.distancias = deque(maxlen=100)
        self.ultimo_tiempo = time.time()
        self.lock = threading.Lock()
        
    def actualizar_desde_mpu(self, gyro_z, dt):
        with self.lock:
            gyro_z_rad = gyro_z * (math.pi / 180.0) / 131.0
            self.theta += gyro_z_rad * dt
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
    
    def actualizar_posicion(self, comando, dt):
        with self.lock:
            if comando == "avanzar":
                velocidad_estimada = 0.3
                dx = velocidad_estimada * dt * math.cos(self.theta)
                dy = velocidad_estimada * dt * math.sin(self.theta)
                self.x += dx
                self.y += dy
            elif comando == "retroceder":
                velocidad_estimada = 0.3
                dx = -velocidad_estimada * dt * math.cos(self.theta)
                dy = -velocidad_estimada * dt * math.sin(self.theta)
                self.x += dx
                self.y += dy
            
            self.historial_x.append(self.x)
            self.historial_y.append(self.y)
    
    def agregar_distancia(self, distancia):
        if distancia > 0 and distancia < 2000:
            with self.lock:
                self.distancias.append((self.x, self.y, self.theta, distancia / 1000.0))
    
    def obtener_estado(self):
        with self.lock:
            return {
                'x': self.x,
                'y': self.y,
                'theta': self.theta,
                'historial_x': list(self.historial_x),
                'historial_y': list(self.historial_y),
                'distancias': list(self.distancias)
            }
    
    def reset(self):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.historial_x.clear()
            self.historial_y.clear()
            self.distancias.clear()

odometria = Odometria()
ultimo_comando = "stop"
marcador_clave = None

# ==================== FUNCIONES DE DETECCIÓN ====================
def coincide_rango(valor, rango):
    return rango[0] <= valor <= rango[1]

def clasificar_marcador(hu_log):
    for figura, reglas in RANGOS_HU.items():
        ok = True
        for idx, rango in reglas.items():
            if not coincide_rango(hu_log[idx-1], rango):
                ok = False
                break
        if ok:
            return figura
    return None

def detectar_marcador(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask_orange = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
    mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
    mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    
    mask = cv2.bitwise_or(mask_orange, mask_green)
    mask = cv2.bitwise_or(mask, mask_yellow)
    
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not cnts:
        return None, None, None, frame
    
    c = max(cnts, key=cv2.contourArea)
    
    if cv2.contourArea(c) < 500:
        return None, None, None, frame
    
    x, y, w, h = cv2.boundingRect(c)
    
    m = cv2.moments(c)
    if m["m00"] > 0:
        hu = cv2.HuMoments(m).flatten()
        hu_log = -np.sign(hu) * np.log10(np.abs(hu) + 1e-30)
        
        tipo = clasificar_marcador(hu_log)
        
        if tipo:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, tipo, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            
            return tipo, cx, cy, frame
    
    return None, None, None, frame

# ==================== COMUNICACIÓN UDP ====================
def enviar_comando(comando, espera_respuesta=True):
    global ultimo_comando
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        sock.sendto(comando.encode('utf-8'), (SERVER_IP, UDP_PORT))
        
        if comando in ["avanzar", "retroceder", "izquierda", "derecha", "stop"]:
            ultimo_comando = comando.split()[0]
        
        if espera_respuesta:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            respuesta = data.decode('utf-8')
        else:
            respuesta = "Enviado"
        
        sock.close()
        return respuesta
    except Exception as e:
        return f"Error: {e}"

# ==================== RECEPCIÓN DE VIDEO ====================
def recibir_video_frontal():
    global frame_frontal, running
    
    max_intentos = 3
    for intento in range(max_intentos):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((SERVER_IP, TCP_PORT_VIDEO_FRONTAL))
            print("[CLIENTE] ✓ Conectado a video frontal")
            break
        except Exception as e:
            print(f"[ERROR] Video frontal (intento {intento+1}): {e}")
            if intento < max_intentos - 1:
                time.sleep(2)
            else:
                print("[WARN] Continuando sin video frontal")
                return
    
    data = b""
    payload_size = struct.calcsize("Q")
    
    while running:
        try:
            while len(data) < payload_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            packed_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_size)[0]
            
            while len(data) < msg_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            if frame is not None:
                frame_frontal = frame.copy()
            
        except Exception as e:
            print(f"[ERROR] Loop video frontal: {e}")
            break
    
    sock.close()

def recibir_video_superior():
    global frame_superior, running
    
    max_intentos = 3
    for intento in range(max_intentos):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((SERVER_IP, TCP_PORT_VIDEO_SUPERIOR))
            print("[CLIENTE] ✓ Conectado a video superior")
            break
        except Exception as e:
            print(f"[ERROR] Video superior (intento {intento+1}): {e}")
            if intento < max_intentos - 1:
                time.sleep(2)
            else:
                print("[WARN] Continuando sin video superior")
                return
    
    data = b""
    payload_size = struct.calcsize("Q")
    
    while running:
        try:
            while len(data) < payload_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            packed_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_size)[0]
            
            while len(data) < msg_size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            if frame is not None:
                frame_superior = frame.copy()
            
        except Exception as e:
            print(f"[ERROR] Loop video superior: {e}")
            break
    
    sock.close()

# ==================== RECEPCIÓN DE ESTADO ====================
def recibir_estado():
    global estado_rover, running
    
    max_intentos = 3
    for intento in range(max_intentos):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((SERVER_IP, TCP_PORT_STATUS))
            print("[CLIENTE] ✓ Conectado a status")
            break
        except Exception as e:
            print(f"[ERROR] Status (intento {intento+1}): {e}")
            if intento < max_intentos - 1:
                time.sleep(2)
            else:
                return
    
    data = b""
    
    while running:
        try:
            while len(data) < 4:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            size = struct.unpack("I", data[:4])[0]
            data = data[4:]
            
            while len(data) < size:
                packet = sock.recv(BUFFER_SIZE)
                if not packet:
                    break
                data += packet
            
            json_data = data[:size]
            data = data[size:]
            
            estado = json.loads(json_data.decode('utf-8'))
            
            with estado_lock:
                estado_rover.update(estado)
            
        except Exception as e:
            print(f"[ERROR] Loop status: {e}")
            break
    
    sock.close()

# ==================== ACTUALIZACIÓN ODOMETRÍA ====================
def actualizar_odometria_thread():
    global ultimo_comando
    ultimo_tiempo = time.time()
    
    while running:
        try:
            tiempo_actual = time.time()
            dt = tiempo_actual - ultimo_tiempo
            ultimo_tiempo = tiempo_actual
            
            with estado_lock:
                if 'gyro' in estado_rover and 'z' in estado_rover['gyro']:
                    try:
                        gyro_z = float(estado_rover['gyro']['z'])
                        odometria.actualizar_desde_mpu(gyro_z, dt)
                    except:
                        pass
                
                if 'distancia' in estado_rover:
                    try:
                        dist_str = estado_rover['distancia']
                        if 'mm' in dist_str:
                            dist = float(dist_str.replace('mm', '').strip())
                            odometria.agregar_distancia(dist)
                    except:
                        pass
            
            if ultimo_comando in ["avanzar", "retroceder"]:
                odometria.actualizar_posicion(ultimo_comando, dt)
            
            time.sleep(0.05)
            
        except Exception as e:
            print(f"[ERROR] Odometría: {e}")
            time.sleep(0.1)

# ==================== INTERFAZ GRÁFICA OPTIMIZADA ====================
class RoverClienteUI:
    def __init__(self, root):
        global marcador_clave
        
        self.root = root
        self.root.title("🤖 Control de Rover - Sistema Completo")
        
        # Configuración responsive
        self.root.state('zoomed')
        self.root.update_idletasks()
        self.root.configure(bg="#1e1e1e")
        
        marcador_clave = tk.StringVar(value="Circulo")
        self.velocidad_actual = 200
        
        # Variables de control de teclado
        self.tecla_activa_actual = None
        self.ultimo_comando_tiempo = 0
        self.debounce_delay = 0.1
        
        self.root.bind('<KeyPress>', self.tecla_presionada)
        self.root.bind('<KeyRelease>', self.tecla_liberada)
        
        # Configurar grid responsive
        self.root.grid_rowconfigure(0, weight=2)  # Fila superior (videos y mapa)
        self.root.grid_rowconfigure(1, weight=1)  # Fila inferior (controles y log)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_columnconfigure(2, weight=1)
        
        self.crear_interfaz()
        self.actualizar_gui()
        
    def agregar_log_ui(self, mensaje):
        """Agrega mensaje al log de la interfaz"""
        timestamp = time.strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] {mensaje}\n"
        
        self.log_text.config(state='normal')
        self.log_text.insert('end', log_msg)
        self.log_text.see('end')
        self.log_text.config(state='disabled')
        
    def crear_interfaz(self):
        # ==================== FILA 0: VIDEOS Y MAPA ====================
        
        # VIDEO FRONTAL (columna 0)
        video_frontal_frame = tk.Frame(self.root, bg="#2d2d2d")
        video_frontal_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        tk.Label(video_frontal_frame, text="🎥 CÁMARA FRONTAL", 
                bg="#2d2d2d", fg="white", font=("Arial", 11, "bold")).pack(pady=3)
        
        self.video_frontal = tk.Label(video_frontal_frame, bg="black")
        self.video_frontal.pack(padx=5, pady=5, fill="both", expand=True)
        
        # VIDEO SUPERIOR (columna 1)
        video_superior_frame = tk.Frame(self.root, bg="#2d2d2d")
        video_superior_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        
        tk.Label(video_superior_frame, text="🎥 CÁMARA SUPERIOR", 
                bg="#2d2d2d", fg="white", font=("Arial", 11, "bold")).pack(pady=3)
        
        self.video_superior = tk.Label(video_superior_frame, bg="black")
        self.video_superior.pack(padx=5, pady=5, fill="both", expand=True)
        
        # MAPA ODOMETRÍA (columna 2)
        map_frame = tk.Frame(self.root, bg="#2d2d2d")
        map_frame.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        
        tk.Label(map_frame, text="🗺️ MAPA DE ODOMETRÍA", 
                bg="#2d2d2d", fg="#00ff00", font=("Arial", 11, "bold")).pack(pady=3)
        
        self.fig = Figure(figsize=(4, 3.5), facecolor='#1e1e1e')
        self.ax = self.fig.add_subplot(111, facecolor='#2d2d2d')
        self.ax.set_xlabel('X (metros)', color='white', fontsize=8)
        self.ax.set_ylabel('Y (metros)', color='white', fontsize=8)
        self.ax.tick_params(colors='white', labelsize=7)
        self.ax.grid(True, alpha=0.3)
        
        self.canvas_mapa = FigureCanvasTkAgg(self.fig, map_frame)
        self.canvas_mapa.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
        
        self.odom_label = tk.Label(map_frame, 
                                   text="Posición: (0.0, 0.0) m | Ángulo: 0.0°", 
                                   bg="#3d3d3d", fg="#00ff00", 
                                   font=("Arial", 8, "bold"))
        self.odom_label.pack(pady=2)
        
        tk.Button(map_frame, text="🔄 Reset", 
                 command=self.reset_odometria,
                 bg="#ff9800", fg="white", font=("Arial", 8, "bold")).pack(pady=2)
        
        # ==================== FILA 1: CONTROLES Y LOG ====================
        
        # CONTROLES (columnas 0-1)
        control_frame = tk.Frame(self.root, bg="#2d2d2d")
        control_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")
        
        # Configurar grid interno para controles en dos columnas
        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=1)
        
        tk.Label(control_frame, text="🎮 PANEL DE CONTROL", 
                bg="#2d2d2d", fg="#00ff00", font=("Arial", 12, "bold")).grid(
                    row=0, column=0, columnspan=2, pady=5)
        
        # COLUMNA IZQUIERDA DE CONTROLES
        col_izq = tk.Frame(control_frame, bg="#2d2d2d")
        col_izq.grid(row=1, column=0, sticky="nsew", padx=5)
        
        # Sensores
        sensores_frame = tk.LabelFrame(col_izq, text="📊 Sensores", 
                                       bg="#3d3d3d", fg="white", font=("Arial", 9, "bold"))
        sensores_frame.pack(fill="x", pady=2)
        
        self.temp_label = tk.Label(sensores_frame, text="🌡️ Temp: --", 
                                   bg="#3d3d3d", fg="#ff6b6b", font=("Arial", 8))
        self.temp_label.pack(anchor="w", padx=5, pady=1)
        
        self.luz_label = tk.Label(sensores_frame, text="💡 Luz: --", 
                                  bg="#3d3d3d", fg="#ffd93d", font=("Arial", 8))
        self.luz_label.pack(anchor="w", padx=5, pady=1)
        
        self.humedad_label = tk.Label(sensores_frame, text="💧 Humedad: --", 
                                      bg="#3d3d3d", fg="#6bcfff", font=("Arial", 8))
        self.humedad_label.pack(anchor="w", padx=5, pady=1)
        
        self.dist_label = tk.Label(sensores_frame, text="📏 Dist: --", 
                                   bg="#3d3d3d", fg="#95e1d3", font=("Arial", 8))
        self.dist_label.pack(anchor="w", padx=5, pady=1)
        
        # Estado
        estado_frame = tk.LabelFrame(col_izq, text="🤖 Estado", 
                                     bg="#3d3d3d", fg="white", font=("Arial", 9, "bold"))
        estado_frame.pack(fill="x", pady=2)
        
        self.modo_label = tk.Label(estado_frame, text="Modo: MANUAL", 
                                   bg="#3d3d3d", fg="#00ff00", font=("Arial", 8))
        self.modo_label.pack(anchor="w", padx=5, pady=1)
        
        self.marcadores_label = tk.Label(estado_frame, text="🎯 Marcadores: 0", 
                                         bg="#3d3d3d", fg="#ff9ff3", font=("Arial", 8))
        self.marcadores_label.pack(anchor="w", padx=5, pady=1)
        
        # Movimiento
        mov_frame = tk.LabelFrame(col_izq, text="🕹️ Movimiento (WASD)", 
                                  bg="#3d3d3d", fg="white", font=("Arial", 9, "bold"))
        mov_frame.pack(fill="x", pady=2)
        
        btn_frame = tk.Frame(mov_frame, bg="#3d3d3d")
        btn_frame.pack(pady=3)
        
        tk.Button(btn_frame, text="▲", width=3, height=1, 
                 command=lambda: self.comando_movimiento("avanzar")).grid(row=0, column=1, padx=2, pady=2)
        
        tk.Button(btn_frame, text="◀", width=3, height=1,
                 command=lambda: self.comando_movimiento("izquierda")).grid(row=1, column=0, padx=2, pady=2)
        tk.Button(btn_frame, text="■", width=3, height=1, bg="#ff4444",
                 command=lambda: self.comando_movimiento("stop")).grid(row=1, column=1, padx=2, pady=2)
        tk.Button(btn_frame, text="▶", width=3, height=1,
                 command=lambda: self.comando_movimiento("derecha")).grid(row=1, column=2, padx=2, pady=2)
        
        tk.Button(btn_frame, text="▼", width=3, height=1,
                 command=lambda: self.comando_movimiento("retroceder")).grid(row=2, column=1, padx=2, pady=2)
        
        # COLUMNA DERECHA DE CONTROLES
        col_der = tk.Frame(control_frame, bg="#2d2d2d")
        col_der.grid(row=1, column=1, sticky="nsew", padx=5)
        
        # Velocidad
        vel_frame = tk.LabelFrame(col_der, text="⚡ Velocidad", 
                                  bg="#3d3d3d", fg="white", font=("Arial", 9, "bold"))
        vel_frame.pack(fill="x", pady=2)
        
        self.vel_label = tk.Label(vel_frame, text=f"{self.velocidad_actual}/255", 
                                 bg="#3d3d3d", fg="white", font=("Arial", 8))
        self.vel_label.pack(pady=1)
        
        self.vel_slider = tk.Scale(vel_frame, from_=0, to=255, orient="horizontal",
                                   bg="#3d3d3d", fg="white", troughcolor="#1e1e1e",
                                   command=self.velocidad_changed, length=150)
        self.vel_slider.set(200)
        self.vel_slider.pack(padx=5, pady=1)
        
        # Servos
        servo_frame = tk.LabelFrame(col_der, text="🔧 Servos", 
                                    bg="#3d3d3d", fg="white", font=("Arial", 9, "bold"))
        servo_frame.pack(fill="x", pady=2)
        
        tk.Label(servo_frame, text="Cam Front (0-30°)", 
                bg="#3d3d3d", fg="white", font=("Arial", 7)).pack(anchor="w", padx=5)
        self.servo3_slider = tk.Scale(servo_frame, from_=0, to=30, orient="horizontal",
                                      bg="#3d3d3d", fg="white", troughcolor="#1e1e1e",
                                      command=self.servo3_changed, length=150)
        self.servo3_slider.set(15)
        self.servo3_slider.pack(padx=5, pady=1)
        
        tk.Label(servo_frame, text="Cam Super (0-180°)", 
                bg="#3d3d3d", fg="white", font=("Arial", 7)).pack(anchor="w", padx=5)
        self.servo4_slider = tk.Scale(servo_frame, from_=0, to=180, orient="horizontal",
                                      bg="#3d3d3d", fg="white", troughcolor="#1e1e1e",
                                      command=self.servo4_changed, length=150)
        self.servo4_slider.set(90)
        self.servo4_slider.pack(padx=5, pady=1)
        
        # Captura
        captura_frame = tk.LabelFrame(col_der, text="📸 Captura", 
                                      bg="#3d3d3d", fg="white", font=("Arial", 9, "bold"))
        captura_frame.pack(fill="x", pady=2)
        
        tk.Button(captura_frame, text="📷 Capturar Foto", 
                 command=self.capturar_imagen_manual, bg="#2196F3", fg="white",
                 font=("Arial", 7, "bold")).pack(padx=5, pady=1, fill="x")
        
        tk.Button(captura_frame, text="💾 Guardar CSV", 
                 command=self.guardar_datos_manual, bg="#4CAF50", fg="white",
                 font=("Arial", 7, "bold")).pack(padx=5, pady=1, fill="x")
        
        # Marcador
        marcador_frame = tk.LabelFrame(col_der, text="🎯 Marcador", 
                                       bg="#3d3d3d", fg="white", font=("Arial", 9, "bold"))
        marcador_frame.pack(fill="x", pady=2)
        
        opciones = ["Cruz", "T", "Circulo", "Triangulo", "Cuadrado"]
        dropdown = ttk.Combobox(marcador_frame, textvariable=marcador_clave, 
                               values=opciones, state="readonly", width=12)
        dropdown.pack(padx=5, pady=2)
        dropdown.current(2)
        
        tk.Button(marcador_frame, text="🎯 Detectar", 
                 command=self.capturar_marcador, bg="#FF9800", fg="white",
                 font=("Arial", 7, "bold")).pack(padx=5, pady=1, fill="x")
        
        # Salir
        tk.Button(col_der, text="✖ SALIR", command=self.cerrar,
                 bg="#d32f2f", fg="white", font=("Arial", 9, "bold"),
                 height=2).pack(side="bottom", fill="x", padx=5, pady=3)
        
        # LOG DE ACCIONES (columna 2)
        log_frame = tk.Frame(self.root, bg="#2d2d2d")
        log_frame.grid(row=1, column=2, padx=5, pady=5, sticky="nsew")
        
        tk.Label(log_frame, text="📋 LOG DE ACCIONES", 
                bg="#2d2d2d", fg="#00ff00", font=("Arial", 11, "bold")).pack(pady=3)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame, 
            wrap=tk.WORD, 
            width=35, 
            height=15,
            bg="#1e1e1e", 
            fg="#00ff00",
            font=("Consolas", 8),
            state='disabled'
        )
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        tk.Button(log_frame, text="🗑️ Limpiar Log", 
                 command=self.limpiar_log, bg="#555", fg="white",
                 font=("Arial", 8)).pack(pady=2)
        
        # Log inicial
        self.agregar_log_ui("✓ Sistema iniciado")
        self.agregar_log_ui("⌨️ Usa WASD o Flechas para controlar")
        self.agregar_log_ui("🎯 Selecciona marcador clave y detecta")
    
    def limpiar_log(self):
        """Limpia el log visual"""
        self.log_text.config(state='normal')
        self.log_text.delete(1.0, 'end')
        self.log_text.config(state='disabled')
        self.agregar_log_ui("Log limpiado")
    
    # ===== CALLBACKS =====
    def velocidad_changed(self, val):
        self.velocidad_actual = int(float(val))
        self.vel_label.config(text=f"{self.velocidad_actual}/255")
        
        def enviar():
            resp = enviar_comando(f"set_velocidad {self.velocidad_actual}")
            self.agregar_log_ui(f"⚡ Velocidad: {self.velocidad_actual} → {resp}")
        
        threading.Thread(target=enviar, daemon=True).start()
    
    def servo3_changed(self, val):
        angulo = int(float(val))
        
        def enviar():
            resp = enviar_comando(f"servo3 {angulo}")
            self.agregar_log_ui(f"🔧 Servo Cam Frontal: {angulo}° → {resp}")
        
        threading.Thread(target=enviar, daemon=True).start()
    
    def servo4_changed(self, val):
        angulo = int(float(val))
        
        def enviar():
            resp = enviar_comando(f"servo4 {angulo}")
            self.agregar_log_ui(f"🔧 Servo Cam Superior: {angulo}° → {resp}")
        
        threading.Thread(target=enviar, daemon=True).start()
    
    def comando_movimiento(self, comando):
        if comando in ["avanzar", "retroceder", "izquierda", "derecha"]:
            cmd = f"{comando} {self.velocidad_actual}"
            icono = {"avanzar": "⬆️", "retroceder": "⬇️", "izquierda": "⬅️", "derecha": "➡️"}[comando]
        else:
            cmd = comando
            icono = "⏸️"
        
        def enviar():
            resp = enviar_comando(cmd)
            self.agregar_log_ui(f"{icono} {comando.upper()}: {resp}")
        
        threading.Thread(target=enviar, daemon=True).start()
    
    def tecla_presionada(self, event):
        tecla = event.keysym.lower()
        tiempo_actual = time.time()
        
        if tiempo_actual - self.ultimo_comando_tiempo < self.debounce_delay:
            return
        
        if self.tecla_activa_actual is not None and self.tecla_activa_actual != tecla:
            return
        
        mapa_comandos = {
            'w': 'avanzar', 'up': 'avanzar',
            's': 'retroceder', 'down': 'retroceder',
            'a': 'izquierda', 'left': 'izquierda',
            'd': 'derecha', 'right': 'derecha'
        }
        
        if tecla in mapa_comandos and self.tecla_activa_actual is None:
            self.tecla_activa_actual = tecla
            self.ultimo_comando_tiempo = tiempo_actual
            self.comando_movimiento(mapa_comandos[tecla])
    
    def tecla_liberada(self, event):
        tecla = event.keysym.lower()
        
        if tecla == self.tecla_activa_actual:
            self.tecla_activa_actual = None
            self.comando_movimiento("stop")
    
    def capturar_imagen_manual(self):
        def ejecutar():
            respuesta = enviar_comando("capturar_imagen")
            self.agregar_log_ui(f"📷 {respuesta}")
            self.root.after(0, lambda: messagebox.showinfo("Captura", respuesta))
        
        threading.Thread(target=ejecutar, daemon=True).start()
    
    def guardar_datos_manual(self):
        def ejecutar():
            respuesta = enviar_comando("guardar_datos")
            self.agregar_log_ui(f"💾 {respuesta}")
            self.root.after(0, lambda: messagebox.showinfo("Guardado", respuesta))
        
        threading.Thread(target=ejecutar, daemon=True).start()
    
    def capturar_marcador(self):
        global frame_frontal
        
        if frame_frontal is None:
            self.agregar_log_ui("⚠️ No hay frame de cámara frontal")
            messagebox.showwarning("Captura", "No hay frame disponible")
            return
        
        tipo, cx, cy, frame_anotado = detectar_marcador(frame_frontal.copy())
        
        if tipo is None:
            self.agregar_log_ui("❌ No se detectó marcador válido")
            messagebox.showinfo("Detección", "No se detectó ningún marcador válido")
            return
        
        h, w = frame_frontal.shape[:2]
        es_clave = (tipo == marcador_clave.get())
        
        comando = f"marcador:{tipo}:{cx}:{cy}:{'true' if es_clave else 'false'}"
        
        def ejecutar():
            respuesta = enviar_comando(comando)
            if es_clave:
                self.agregar_log_ui(f"🎯 MARCADOR CLAVE '{tipo}' detectado")
                self.agregar_log_ui(f"🤖 Iniciando secuencia autónoma...")
                self.root.after(0, lambda: messagebox.showinfo("🎯 Marcador Clave", 
                                  f"Marcador clave '{tipo}' detectado!\nIniciando secuencia autónoma..."))
            else:
                self.agregar_log_ui(f"🎯 Marcador '{tipo}' guardado → {respuesta}")
                self.root.after(0, lambda: messagebox.showinfo("Marcador", f"Marcador '{tipo}' guardado\n{respuesta}"))
        
        threading.Thread(target=ejecutar, daemon=True).start()
    
    def reset_odometria(self):
        odometria.reset()
        self.agregar_log_ui("🔄 Odometría reiniciada")
        messagebox.showinfo("Reset", "Odometría reiniciada")
    
    def actualizar_mapa(self):
        try:
            estado_odom = odometria.obtener_estado()
            
            self.ax.clear()
            self.ax.set_facecolor('#2d2d2d')
            self.ax.grid(True, alpha=0.3, color='white')
            
            if len(estado_odom['historial_x']) > 1:
                self.ax.plot(estado_odom['historial_x'], 
                           estado_odom['historial_y'], 
                           'cyan', linewidth=2, label='Trayectoria')
            
            x, y, theta = estado_odom['x'], estado_odom['y'], estado_odom['theta']
            
            arrow_len = 0.3
            dx = arrow_len * math.cos(theta)
            dy = arrow_len * math.sin(theta)
            self.ax.arrow(x, y, dx, dy, head_width=0.15, head_length=0.1,
                         fc='lime', ec='lime', linewidth=2)
            
            for ox, oy, otheta, dist in estado_odom['distancias'][-20:]:
                obs_x = ox + dist * math.cos(otheta)
                obs_y = oy + dist * math.sin(otheta)
                self.ax.plot(obs_x, obs_y, 'ro', markersize=3, alpha=0.6)
            
            max_range = max(3.0, 
                          max(abs(x), abs(y)) + 1 if x != 0 or y != 0 else 3.0)
            self.ax.set_xlim(-max_range, max_range)
            self.ax.set_ylim(-max_range, max_range)
            self.ax.set_xlabel('X (m)', color='white', fontsize=8)
            self.ax.set_ylabel('Y (m)', color='white', fontsize=8)
            self.ax.tick_params(colors='white', labelsize=7)
            self.ax.legend(loc='upper right', facecolor='#3d3d3d', 
                          edgecolor='white', labelcolor='white', fontsize=7)
            
            self.canvas_mapa.draw()
            
            angulo_grados = math.degrees(theta)
            self.odom_label.config(
                text=f"Pos: ({x:.2f}, {y:.2f}) m | Ángulo: {angulo_grados:.1f}°"
            )
            
        except Exception as e:
            pass
    
    def actualizar_gui(self):
        global frame_frontal, frame_superior, estado_rover
        
        # Tamaño de video adaptable
        root_width = self.root.winfo_width()
        if root_width <= 1:
            video_width = 400
        else:
            video_width = min(500, int(root_width * 0.28))
        
        video_width = max(300, video_width)
        video_height = int(video_width * 0.75)
        
        # Actualizar video frontal
        if frame_frontal is not None:
            frame = frame_frontal.copy()
            tipo, cx, cy, frame_anotado = detectar_marcador(frame)
            
            frame_rgb = cv2.cvtColor(frame_anotado, cv2.COLOR_BGR2RGB)
            frame_rgb = cv2.resize(frame_rgb, (video_width, video_height))
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_frontal.imgtk = imgtk
            self.video_frontal.configure(image=imgtk)
        
        # Actualizar video superior
        if frame_superior is not None:
            frame_rgb = cv2.cvtColor(frame_superior, cv2.COLOR_BGR2RGB)
            frame_rgb = cv2.resize(frame_rgb, (video_width, video_height))
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_superior.imgtk = imgtk
            self.video_superior.configure(image=imgtk)
        
        # Actualizar estado
        with estado_lock:
            if estado_rover:
                self.temp_label.config(text=f"🌡️ {estado_rover.get('temperatura', 'N/A')}")
                self.luz_label.config(text=f"💡 {estado_rover.get('luz', 'N/A')}")
                self.humedad_label.config(text=f"💧 {estado_rover.get('humedad', 'N/A')}")
                self.dist_label.config(text=f"📏 {estado_rover.get('distancia', 'N/A')}")
                
                modo = estado_rover.get('modo', 'manual').upper()
                color = "#ff9900" if modo == "AUTONOMO" else "#00ff00"
                self.modo_label.config(text=f"Modo: {modo}", fg=color)
                
                marcadores = estado_rover.get('marcadores_detectados', 0)
                self.marcadores_label.config(text=f"🎯 Marcadores: {marcadores}")
                
                ultimo_log = estado_rover.get('ultimo_log', '')
                if ultimo_log and hasattr(self, '_ultimo_log_visto'):
                    if ultimo_log != self._ultimo_log_visto:
                        self.agregar_log_ui(f"📡 Servidor: {ultimo_log}")
                        self._ultimo_log_visto = ultimo_log
                elif not hasattr(self, '_ultimo_log_visto'):
                    self._ultimo_log_visto = ultimo_log
        
        # Actualizar mapa
        self.actualizar_mapa()
        
        self.root.after(50, self.actualizar_gui)
    
    def cerrar(self):
        global running
        self.agregar_log_ui("🔴 Cerrando sistema...")
        running = False
        self.root.destroy()

# ==================== MAIN ====================
if __name__ == "__main__":
    print("=" * 60)
    print("   🤖 CLIENTE ROVER - INTERFAZ OPTIMIZADA")
    print("=" * 60)
    print(f"\n🔗 Conectando a servidor: {SERVER_IP}")
    
    threading.Thread(target=recibir_video_frontal, daemon=True).start()
    threading.Thread(target=recibir_video_superior, daemon=True).start()
    threading.Thread(target=recibir_estado, daemon=True).start()
    threading.Thread(target=actualizar_odometria_thread, daemon=True).start()
    
    time.sleep(1)
    
    root = tk.Tk()
    app = RoverClienteUI(root)
    
    root.protocol("WM_DELETE_WINDOW", app.cerrar)
    root.mainloop()
    

    print("\n[CLIENTE] Desconectado")
