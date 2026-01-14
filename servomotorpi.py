import pigpio
import time
import subprocess

# Comando para iniciar pigpio en segundo plano
comando = ["sudo", "pigpiod"]

# Ejecutar el comando y capturar la salida
resultado = subprocess.run(comando, text=True, capture_output=True)

# Mostrar la salida
print(resultado.stdout)

# Si ocurre algún error, lo mostramos
if resultado.stderr:
    print("Error:", resultado.stderr)

# Inicializa pigpio
pi = pigpio.pi()

# Verifica si pigpio está corriendo
if not pi.connected:
    print("No se pudo conectar al demonio pigpio.")
    exit()

# Define el pin GPIO 12 donde se conecta el servomotor
gpio_pin = 12

# Configura el GPIO 12 como salida PWM
pi.set_mode(gpio_pin, pigpio.OUTPUT)

# Función para mover el servomotor a un ángulo
def servo(angulo):
    angulo=int(max(1, min(130, angulo)))
    # Convierte el ángulo (0-180) a un pulso en microsegundos (500-2500)
    pulso = ((angulo / 180) * (2500 - 500) + 500)
    pi.set_servo_pulsewidth(gpio_pin, pulso)

    #dutycycle = int((pulso / 20000) * 1000000)
    #pi.hardware_PWM(gpio_pin, 50, dutycycle) #0-1000000 0-100% 50hz 20ms 0.5ms-2.5ms 500us-2500us 2.5%-12.5% 25000-125000

# Función para mover el servomotor suavemente de un ángulo a otro
def servo2(angulo_inicial, angulo_final, pasos=100, tiempo_entre_pasos=0.003):
    """ Mueve el servomotor de angulo_inicial a angulo_final suavemente en pasos """
    paso = (angulo_final - angulo_inicial) / pasos  # Calcula el tamaño de cada paso
    
    for i in range(pasos):
        angulo_actual = angulo_inicial + paso * i
        servo(angulo_actual)  # Mueve el servomotor al ángulo calculado
        time.sleep(tiempo_entre_pasos)  # Espera entre cada paso

#servo(70)
#time.sleep(1)  # Espera 1 segundo

# Bucle principal
try:
    while True:	
        servo(1)
        time.sleep(2)
        servo(120)
        time.sleep(2)
        
finally:
    # Detén el servomotor y apaga la conexión de pigpio
    pi.set_servo_pulsewidth(gpio_pin, 0)
    #pi.hardware_PWM(gpio_pin, 0, 0)
    pi.stop()
