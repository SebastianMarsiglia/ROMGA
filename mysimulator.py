import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Parámetros del robot
radio_rueda = 0.1651 / 2         # metros (diámetro 16.51 cm)
L = 0.53                         # distancia entre ruedas (m)
C = 2 * np.pi * radio_rueda      # circunferencia de la rueda (m)
ancho_robot = L
largo_robot = 0.8                # largo del robot (m)

# Estado inicial
x, y, theta, d = 0.0, 0.0, 0.0, 0.0
x_hist, y_hist = [x], [y]
t_anterior = time.time()
t_inicio = time.time()

# Función ficticia para obtener RPMs (puedes reemplazar con lectura real)
def obtener_rpms():
    rpm_izq = 135   #-500 a 500 rpm
    rpm_der = -150   #-500 a 500 rpm
    return rpm_izq, rpm_der

# Función para actualizar la animación
def actualizar_estado(frame):
    global x, y, theta, d, t_anterior, x_hist, y_hist, t_inicio

    # Lectura de RPMs
    rpm_izq, rpm_der = obtener_rpms()
    v_izq = (rpm_izq * C) / 60
    v_der = -(rpm_der * C) / 60

    # Cinemática diferencial
    v = (v_der + v_izq) / 2
    w = (v_der - v_izq) / L

    # Tiempo
    t_actual = time.time()
    dt = t_actual - t_anterior
    t_anterior = t_actual

    # Actualización de estado
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += w * dt
    d += v * dt #Distancia recorrida en m
 
    x_hist.append(x)
    y_hist.append(y)
    
    max_hist = 100
    x_hist = x_hist[-max_hist:]
    y_hist = y_hist[-max_hist:]

    # LIMPIAR y dibujar
    ax.clear()
    ax.plot(x_hist, y_hist, 'b-', label="Trayectoria")

    # Dirección del robot
    flecha_longitud = 0.5
    dx = flecha_longitud * np.cos(theta)
    dy = flecha_longitud * np.sin(theta)
    ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc='red', ec='red', label="Orientación")     
    
    # Cálculo de puntos clave del cuerpo del robot
    half_L = L / 2
    half_l = largo_robot / 2

    # Centro delantero y trasero
    front = np.array([x + half_l*np.cos(theta), y + half_l*np.sin(theta)])
    rear  = np.array([x - half_l*np.cos(theta), y - half_l*np.sin(theta)])

    # Offset lateral
    offset_right = np.array([half_L*np.cos(theta + np.pi/2), half_L*np.sin(theta + np.pi/2)])
    offset_left  = -offset_right

    # Esquinas del rectángulo del robot
    front_left  = front + offset_left
    front_right = front + offset_right
    rear_left   = rear + offset_left
    rear_right  = rear + offset_right

    # Dibujar cuerpo del robot
    corners_x = [front_left[0], front_right[0], rear_right[0], rear_left[0], front_left[0]]
    corners_y = [front_left[1], front_right[1], rear_right[1], rear_left[1], front_left[1]]
    ax.plot(corners_x, corners_y, 'k-')

    # Línea entre ruedas
    rueda_izq = x + half_L*np.cos(theta + np.pi/2), y + half_L*np.sin(theta + np.pi/2)
    rueda_der = x - half_L*np.cos(theta + np.pi/2), y - half_L*np.sin(theta + np.pi/2)
    ax.plot([rueda_izq[0], rueda_der[0]], [rueda_izq[1], rueda_der[1]], 'k-', linewidth=2)

    # Ruedas como puntos
    ax.plot(*rueda_izq, 'ko', markersize=8)
    ax.plot(*rueda_der, 'ko', markersize=8)

    # Texto de métricas
    t_sim = t_actual - t_inicio
    
    texto = (
        f"x: {x:.2f} m\ny: {y:.2f} m\n"
        f"θ: {np.degrees(theta)%360:.1f}°\n"
        f"w: {np.degrees(w):.2f} °/s\n"
        f"v: {v:.2f} m/s ({v*3.6:.2f} km/h)\n"
        f"d: {d:.2f} m ({d/1000:.3f} km)\n"
        f"t: {t_sim:.1f} s ({t_sim/60:.1f} min)\n"
        f"dt: {dt:.2f} s\n"
        f"RI: {rpm_izq:.2f} RD: {rpm_der:.2f} rpm\n"

        f"V: {37+np.sin(t_sim)/10:.2f} V\n"
        f"I: {10+np.cos(t_sim)/10:.2f} A  P: {(37+np.sin(t_sim)/10)*(10+np.cos(t_sim)/10):.2f} W\n"
        f"E: {370*t_sim/3600:.2f} Wh\n"
        f"Peso: {140+np.cos(t_sim):.2f} kg\n"
        f"Pendiente: {4+np.cos(t_sim):.2f} °"
    )
        
    ax.text(0.02, 0.95, texto, transform=ax.transAxes, fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Opciones de gráfico
    ax.set_title("Simulación de Odometría del Robot Diferencial")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc='lower right')

# Iniciar animación
fig, ax = plt.subplots()
ani = FuncAnimation(fig, actualizar_estado, interval=50)
plt.show()
