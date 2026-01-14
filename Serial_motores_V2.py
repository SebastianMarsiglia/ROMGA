import serial
import struct
import time

START_FRAME = 0xABCD  # Definición del valor inicial (start frame) que indica el comienzo de un paquete de datos para garantizar una comunicación serial confiable.
TIME_SEND = 0.05    # Intervalo de tiempo (en segundos) entre el envío de comandos (50 ms).
MAX_STEER = 500  # Límite máximo para la dirección (steer).
MAX_SPEED = 500  # Límite máximo para la velocidad (speed).

# Inicialización de la conexión serial
try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=2)  # Configuración del puerto serial de la Raspberry Pi a 115200 baudios con un timeout de 1 segundo.
    # Inicializa el puerto y limpia el buffer
    ser.reset_input_buffer()

except serial.SerialException as e:
    # Si ocurre un error al abrir el puerto, se muestra el mensaje de error y se detiene el programa.
    print(f"Error al abrir el puerto serial: {e}")
    exit(1)

# Función para calcular el checksum
def calculate_checksum(start, steer, speed):
    # El checksum se calcula como la operación XOR entre los valores de start, steer, y speed,
    # y se limita a 16 bits sin signo (0xFFFF) para garantizar que no exceda este rango.
    return (start ^ steer ^ speed) & 0xFFFF

# Función para enviar los comandos a través de UART
def send_command(steer, speed):
    steer = int(max(-MAX_STEER, min(steer, MAX_STEER)))  # Asegurarse de que steer esté entre -MAX_STEER y MAX_STEER.
    speed = int(max(-MAX_SPEED, min(speed, MAX_SPEED)))  # Asegurarse de que speed esté entre -MAX_SPEED y MAX_SPEED.

    # Calcular el checksum para detectar posibles errores en la transmisión de datos.
    checksum = calculate_checksum(START_FRAME, steer, speed)

    # Crear el paquete de datos. El formato '<HhhH' especifica:
    # - H: unsigned short (2 bytes) para el start frame.
    # - h: signed short (2 bytes) para steer (dirección).
    # - h: signed short (2 bytes) para speed (velocidad).
    # - H: unsigned short (2 bytes) para el checksum.
    command = struct.pack('<HhhH', START_FRAME, steer, speed, checksum)

    # Enviar el paquete de datos a través del puerto UART.
    try:
        ser.write(command)
    except serial.SerialException as e:
        # Si ocurre un error durante la comunicación serial, se muestra un mensaje de error.
        print(f"Error de comunicación serial: {e}")
        
        
# Estructura para los datos de feedback
def parse_feedback(data):
    # El formato sigue el patrón: <HhhhhhhHH
    feedback = struct.unpack('<HhhhhhhHH', data)
    start, cmd1, cmd2, speedR_meas, speedL_meas, batVoltage, boardTemp, cmdLed, checksum = feedback
    calc_checksum = (start ^ cmd1 ^ cmd2 ^ speedR_meas ^ speedL_meas ^ batVoltage ^ boardTemp ^ cmdLed) & 0xFFFF

    # Verificar validez de los datos
    if start == START_FRAME and calc_checksum == checksum:
        print(f"Received: start={start}, cmd1={cmd1}, cmd2={cmd2}, speedR_meas={speedR_meas}, "
              f"speedL_meas={speedL_meas}, batVoltage={batVoltage}, boardTemp={boardTemp}, cmdLed={cmdLed}, "
              f"received_checksum={checksum}, calculated_checksum={calc_checksum}")
    #else:
        #print("Datos no validos o no sincronizados")

# Función para recibir datos
def receive_feedback():
    if ser.in_waiting >= 18:  # 18 bytes para toda la estructura
        data = ser.read(18)
        parse_feedback(data)

# Variables iniciales
iSpeed = 0  # Inicializa la velocidad en 0.
iSteer = 0  # Inicializa la dirección en 0.
last_send_time = 0  # Marca el tiempo inicial de envío.

try:
    # Bucle principal optimizado
    while True:
        # Recibir datos
        receive_feedback()
        if ser.is_open:
            ser.reset_input_buffer()
        
        current_time = time.time()

        # Si ha pasado el tiempo definido en TIME_SEND (50 ms), se envía un nuevo comando.
        if current_time - last_send_time >= TIME_SEND:
            send_command(iSteer, iSpeed)  # Enviar los valores actuales de dirección y velocidad.
            last_send_time = current_time  # Actualizar el tiempo del último envío.

        iSteer = 0 # Establecer la dirección a (valor arbitrario). Se suma la mitad al motor izquierdo, se resta la mitad al motor derecho.
        iSpeed = 150  # Establecer la velocidad a (valor arbitrario). Se aplica directo a ambos motores, positivo para adelante, negativo para reversa.
        #iSteer = int(max(-MAX_STEER, min(iSteer, MAX_STEER)))  # Asegurarse de que steer esté entre -MAX_STEER y MAX_STEER.
        #iSpeed = int(max(-MAX_SPEED, min(iSpeed, MAX_SPEED)))  # Asegurarse de que speed esté entre -MAX_SPEED y MAX_SPEED.
        
        # Un retardo de 100 ms es suficiente para evitar ciclos innecesarios y mantener el programa eficiente.
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nPrograma detenido por el usuario.")
finally:
    if ser.is_open:
        ser.close()
        print("Puerto serial cerrado correctamente.")
